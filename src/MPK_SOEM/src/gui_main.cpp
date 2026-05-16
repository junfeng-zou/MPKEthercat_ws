#include "mpk_soem/four_motor_master.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <array>
#include <atomic>
#include <cstdint>
#include <exception>
#include <memory>
#include <thread>

namespace
{

using mpk_soem::MasterOptions;
using mpk_soem::MasterSnapshot;
using mpk_soem::cia402::Mode;

constexpr int kMotorCount = mpk_soem::kMaxMotors;

Mode mode_from_combo(const QComboBox * combo)
{
  return static_cast<Mode>(combo->currentData().toInt());
}

QString hex16(uint16_t value)
{
  return QString("0x%1").arg(value, 4, 16, QLatin1Char('0'));
}

class MainWindow : public QMainWindow
{
public:
  MainWindow()
  {
    setWindowTitle("MPK SOEM Motor Console");
    resize(1180, 720);

    auto * central = new QWidget(this);
    auto * root = new QGridLayout(central);
    root->setColumnStretch(0, 0);
    root->setColumnStretch(1, 1);

    build_connection_panel(root);
    build_command_panel(root);
    build_status_panel(root);
    build_log_panel(root);

    setCentralWidget(central);

    poll_timer_.setInterval(100);
    connect(&poll_timer_, &QTimer::timeout, this, [this]() { poll_status(); });
    poll_timer_.start();
  }

  ~MainWindow() override
  {
    stop_controller();
    join_worker_if_done(true);
  }

private:
  void build_connection_panel(QGridLayout * root)
  {
    auto * box = new QGroupBox("Connection");
    auto * layout = new QGridLayout(box);

    ifname_ = new QLineEdit("eno1");
    rate_hz_ = new QDoubleSpinBox;
    rate_hz_->setRange(1.0, 4000.0);
    rate_hz_->setDecimals(0);
    rate_hz_->setValue(1000.0);

    duration_s_ = new QDoubleSpinBox;
    duration_s_->setRange(0.0, 86400.0);
    duration_s_->setDecimals(1);
    duration_s_->setSuffix(" s");
    duration_s_->setSpecialValueText("forever");
    duration_s_->setValue(0.0);

    no_dc_ = new QCheckBox("Disable DC SYNC0");
    no_id_check_ = new QCheckBox("Skip ID check");

    start_button_ = new QPushButton("Start OP");
    stop_button_ = new QPushButton("Stop");
    stop_button_->setEnabled(false);

    layout->addWidget(new QLabel("Interface"), 0, 0);
    layout->addWidget(ifname_, 0, 1);
    layout->addWidget(new QLabel("Cycle"), 1, 0);
    layout->addWidget(rate_hz_, 1, 1);
    layout->addWidget(new QLabel("Duration"), 2, 0);
    layout->addWidget(duration_s_, 2, 1);
    layout->addWidget(no_dc_, 3, 0, 1, 2);
    layout->addWidget(no_id_check_, 4, 0, 1, 2);
    layout->addWidget(start_button_, 5, 0);
    layout->addWidget(stop_button_, 5, 1);

    connect(start_button_, &QPushButton::clicked, this, [this]() { start_controller(); });
    connect(stop_button_, &QPushButton::clicked, this, [this]() { stop_controller(); });

    root->addWidget(box, 0, 0);
  }

  void build_command_panel(QGridLayout * root)
  {
    auto * box = new QGroupBox("Control");
    auto * layout = new QGridLayout(box);

    mode_ = new QComboBox;
    mode_->addItem("Idle / No mode", static_cast<int>(Mode::NoMode));
    mode_->addItem("PP - Profile Position", static_cast<int>(Mode::ProfilePosition));
    mode_->addItem("PV - Profile Velocity", static_cast<int>(Mode::ProfileVelocity));
    mode_->addItem("CSP - Cyclic Sync Position", static_cast<int>(Mode::Csp));
    mode_->addItem("CSV - Cyclic Sync Velocity", static_cast<int>(Mode::Csv));

    enable_drives_ = new QCheckBox("Enable selected drives");
    enable_drives_->setChecked(false);

    csp_speed_ = new QDoubleSpinBox;
    csp_speed_->setRange(0.0, 10000000.0);
    csp_speed_->setDecimals(0);
    csp_speed_->setSpecialValueText("no ramp");
    csp_speed_->setSuffix(" counts/s");

    layout->addWidget(new QLabel("Mode"), 0, 0);
    layout->addWidget(mode_, 0, 1, 1, 3);
    layout->addWidget(enable_drives_, 1, 0, 1, 4);
    layout->addWidget(new QLabel("CSP ramp"), 2, 0);
    layout->addWidget(csp_speed_, 2, 1, 1, 3);

    for (int i = 0; i < kMotorCount; ++i) {
      motor_enable_[i] = new QCheckBox(QString("M%1").arg(i + 1));
      motor_enable_[i]->setChecked(i == 0);
      target_[i] = new QSpinBox;
      target_[i]->setRange(INT32_MIN, INT32_MAX);
      target_[i]->setSingleStep(100);
      target_[i]->setValue(0);

      layout->addWidget(motor_enable_[i], 3 + i, 0);
      layout->addWidget(new QLabel("Target"), 3 + i, 1);
      layout->addWidget(target_[i], 3 + i, 2, 1, 2);
    }

    apply_button_ = new QPushButton("Apply Command");
    disable_button_ = new QPushButton("Disable All");
    fault_reset_button_ = new QPushButton("Fault Reset Selected");

    layout->addWidget(apply_button_, 7, 0, 1, 4);
    layout->addWidget(disable_button_, 8, 0, 1, 2);
    layout->addWidget(fault_reset_button_, 8, 2, 1, 2);

    connect(apply_button_, &QPushButton::clicked, this, [this]() { apply_command(); });
    connect(disable_button_, &QPushButton::clicked, this, [this]() {
      enable_drives_->setChecked(false);
      apply_command();
    });
    connect(fault_reset_button_, &QPushButton::clicked, this, [this]() { fault_reset_selected(); });

    root->addWidget(box, 1, 0);
  }

  void build_status_panel(QGridLayout * root)
  {
    auto * box = new QGroupBox("Drive Status");
    auto * layout = new QVBoxLayout(box);

    summary_ = new QLabel("Stopped");
    table_ = new QTableWidget(kMotorCount, 10);
    table_->setHorizontalHeaderLabels({
      "Motor", "Enabled", "Status", "State", "Position", "Velocity",
      "Mode", "Phys Pos mm", "Phys Vel mm/s", "Target"});
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionMode(QAbstractItemView::NoSelection);

    for (int row = 0; row < kMotorCount; ++row) {
      set_cell(row, 0, QString("M%1").arg(row + 1));
    }

    layout->addWidget(summary_);
    layout->addWidget(table_);
    root->addWidget(box, 0, 1, 2, 1);
  }

  void build_log_panel(QGridLayout * root)
  {
    auto * box = new QGroupBox("Event Log");
    auto * layout = new QVBoxLayout(box);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(500);
    layout->addWidget(log_);
    root->addWidget(box, 2, 0, 1, 2);
  }

  void start_controller()
  {
    join_worker_if_done(false);
    if (worker_.joinable()) {
      append_log("Controller is already running.");
      return;
    }

    MasterOptions options;
    options.ifname = ifname_->text().trimmed().toStdString();
    options.mode = mode_from_combo(mode_);
    options.targets = targets_from_ui();
    options.enable_drives = enable_drives_->isChecked();
    options.enable_motor_mask = enable_mask_from_ui();
    options.use_distributed_clocks = !no_dc_->isChecked();
    options.require_expected_identity = !no_id_check_->isChecked();
    options.duration_s = duration_s_->value();
    options.csp_speed_counts_per_s = csp_speed_->value();
    options.cycle_time_ns = static_cast<int64_t>(1000000000.0 / rate_hz_->value());
    options.print_every_cycles = static_cast<int>(rate_hz_->value());

    if (options.ifname.empty()) {
      append_log("Interface name is empty.");
      return;
    }

    stop_requested_.store(false);
    worker_done_.store(false);
    master_ = std::make_shared<mpk_soem::FourMotorMaster>(options);
    set_running_ui(true);
    append_log("Starting controller...");

    auto master = master_;
    worker_ = std::thread([this, master]() {
      int rc = 1;
      try {
        if (master->initialize()) {
          rc = master->run(stop_requested_);
        }
      } catch (const std::exception & e) {
        post_log(QString("Controller exception: %1").arg(e.what()));
      }
      post_log(QString("Controller stopped with code %1.").arg(rc));
      worker_done_.store(true);
    });
  }

  void stop_controller()
  {
    stop_requested_.store(true);
    append_log("Stop requested.");
  }

  void apply_command()
  {
    auto master = master_;
    if (!master) {
      append_log("No active controller.");
      return;
    }
    master->set_runtime_command(
      mode_from_combo(mode_),
      targets_from_ui(),
      enable_drives_->isChecked(),
      enable_mask_from_ui(),
      csp_speed_->value());
    append_log("Command applied.");
  }

  void fault_reset_selected()
  {
    auto master = master_;
    if (!master) {
      append_log("No active controller.");
      return;
    }
    const uint8_t mask = enable_mask_from_ui();
    master->request_fault_reset(mask);
    append_log(QString("Fault reset requested for mask 0x%1.").arg(mask, 0, 16));
  }

  void poll_status()
  {
    join_worker_if_done(false);
    auto master = master_;
    if (!master) {
      return;
    }

    const MasterSnapshot snap = master->snapshot();
    summary_->setText(QString("OP=%1  WKC=%2/%3  Mode=%4  Enable mask=0x%5")
        .arg(snap.in_operational ? "yes" : "no")
        .arg(snap.last_wkc)
        .arg(snap.expected_wkc)
        .arg(mpk_soem::cia402::to_string(snap.mode))
        .arg(snap.enable_motor_mask, 0, 16));

    for (int row = 0; row < kMotorCount; ++row) {
      const auto & sample = snap.samples[row];
      const bool enabled = (snap.enable_motor_mask & (1u << row)) != 0;
      const auto state = mpk_soem::cia402::decode_status_word(sample.status_word);
      set_cell(row, 1, enabled ? "yes" : "no");
      set_cell(row, 2, hex16(sample.status_word));
      set_cell(row, 3, mpk_soem::cia402::to_string(state));
      set_cell(row, 4, QString::number(sample.actual_position));
      set_cell(row, 5, QString::number(sample.actual_velocity));
      set_cell(row, 6, QString::number(sample.mode_display));
      set_cell(row, 7, QString::number(sample.physical_position_mm, 'f', 3));
      set_cell(row, 8, QString::number(sample.physical_velocity_mm_s, 'f', 3));
      set_cell(row, 9, QString::number(snap.targets[row]));
      color_row(row, state);
    }
  }

  void join_worker_if_done(bool force)
  {
    if (!worker_.joinable()) {
      return;
    }
    if (force || worker_done_.load()) {
      worker_.join();
      set_running_ui(false);
      if (worker_done_.load()) {
        master_.reset();
      }
    }
  }

  void set_running_ui(bool running)
  {
    start_button_->setEnabled(!running);
    stop_button_->setEnabled(running);
    ifname_->setEnabled(!running);
    rate_hz_->setEnabled(!running);
    duration_s_->setEnabled(!running);
    no_dc_->setEnabled(!running);
    no_id_check_->setEnabled(!running);
  }

  std::array<int32_t, kMotorCount> targets_from_ui() const
  {
    std::array<int32_t, kMotorCount> targets{};
    for (int i = 0; i < kMotorCount; ++i) {
      targets[i] = target_[i]->value();
    }
    return targets;
  }

  uint8_t enable_mask_from_ui() const
  {
    uint8_t mask = 0;
    for (int i = 0; i < kMotorCount; ++i) {
      if (motor_enable_[i]->isChecked()) {
        mask |= static_cast<uint8_t>(1u << i);
      }
    }
    return mask;
  }

  void set_cell(int row, int column, const QString & text)
  {
    auto * item = table_->item(row, column);
    if (!item) {
      item = new QTableWidgetItem;
      table_->setItem(row, column, item);
    }
    item->setText(text);
  }

  void color_row(int row, mpk_soem::cia402::DriveState state)
  {
    QColor bg = Qt::white;
    if (state == mpk_soem::cia402::DriveState::OperationEnabled) {
      bg = QColor(224, 247, 232);
    } else if (state == mpk_soem::cia402::DriveState::Fault ||
      state == mpk_soem::cia402::DriveState::FaultReactionActive)
    {
      bg = QColor(255, 226, 226);
    } else if (state == mpk_soem::cia402::DriveState::SwitchOnDisabled) {
      bg = QColor(239, 243, 248);
    } else if (state == mpk_soem::cia402::DriveState::SwitchedOn ||
      state == mpk_soem::cia402::DriveState::ReadyToSwitchOn)
    {
      bg = QColor(255, 246, 214);
    }

    for (int column = 0; column < table_->columnCount(); ++column) {
      if (auto * item = table_->item(row, column)) {
        item->setBackground(bg);
      }
    }
  }

  void append_log(const QString & text)
  {
    const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    log_->appendPlainText(QString("[%1] %2").arg(stamp, text));
  }

  void post_log(const QString & text)
  {
    QMetaObject::invokeMethod(this, [this, text]() { append_log(text); }, Qt::QueuedConnection);
  }

  QLineEdit * ifname_{nullptr};
  QDoubleSpinBox * rate_hz_{nullptr};
  QDoubleSpinBox * duration_s_{nullptr};
  QCheckBox * no_dc_{nullptr};
  QCheckBox * no_id_check_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};

  QComboBox * mode_{nullptr};
  QCheckBox * enable_drives_{nullptr};
  QDoubleSpinBox * csp_speed_{nullptr};
  std::array<QCheckBox *, kMotorCount> motor_enable_{};
  std::array<QSpinBox *, kMotorCount> target_{};
  QPushButton * apply_button_{nullptr};
  QPushButton * disable_button_{nullptr};
  QPushButton * fault_reset_button_{nullptr};

  QLabel * summary_{nullptr};
  QTableWidget * table_{nullptr};
  QPlainTextEdit * log_{nullptr};
  QTimer poll_timer_;

  std::shared_ptr<mpk_soem::FourMotorMaster> master_;
  std::thread worker_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool worker_done_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  return app.exec();
}
