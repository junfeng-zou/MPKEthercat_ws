#include "mpk_soem/four_motor_master.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

extern "C" {
#if __has_include("ethercat.h")
#include "ethercat.h"
#else
#include "soem/soem.h"
#endif
}

namespace mpk_soem
{
namespace
{

constexpr int kEcTimeoutMon = 500;
constexpr int kEcTimeoutState = 50000;
constexpr size_t kIoMapSize = 4096;

struct __attribute__((packed)) RxPdo
{
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int8_t mode_of_operation;
};

struct __attribute__((packed)) TxPdo
{
  uint16_t status_word;
  int32_t actual_position;
  int32_t actual_velocity;
  int8_t mode_display;
};

static_assert(sizeof(RxPdo) == 11, "Unexpected RxPDO size");
static_assert(sizeof(TxPdo) == 11, "Unexpected TxPDO size");

bool mode_is_position(cia402::Mode mode)
{
  return mode == cia402::Mode::Csp || mode == cia402::Mode::ProfilePosition;
}

bool mode_is_velocity(cia402::Mode mode)
{
  return mode == cia402::Mode::Csv || mode == cia402::Mode::ProfileVelocity;
}

void add_ns(timespec & t, int64_t ns)
{
  t.tv_nsec += ns;
  while (t.tv_nsec >= 1000000000L) {
    t.tv_nsec -= 1000000000L;
    ++t.tv_sec;
  }
}

}  // namespace

struct FourMotorMaster::SoemState
{
  ecx_contextt context{};
};

FourMotorMaster::FourMotorMaster(MasterOptions options)
: options_(std::move(options)), soem_(std::make_unique<SoemState>()), iomap_(kIoMapSize, 0)
{
  if (options_.motor_count < 1 || options_.motor_count > kMaxMotors) {
    throw std::invalid_argument("motor_count must be 1..4");
  }
  if (options_.cycle_time_ns <= 0) {
    throw std::invalid_argument("cycle_time_ns must be positive");
  }
}

FourMotorMaster::~FourMotorMaster()
{
  shutdown();
}

bool FourMotorMaster::initialize()
{
  std::cout << "Opening SOEM interface '" << options_.ifname << "'..." << std::endl;
  auto * context = &soem_->context;
  if (ecx_init(context, options_.ifname.c_str()) <= 0) {
    std::cerr << "ec_init failed. Run as root and check the interface name." << std::endl;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = true;
  }

  if (ecx_config_init(context) <= 0) {
    std::cerr << "No EtherCAT slaves found." << std::endl;
    return false;
  }
  std::cout << context->slavecount << " slave(s) found." << std::endl;

  if (context->slavecount < options_.motor_count) {
    std::cerr << "Need " << options_.motor_count << " motor slaves, found " << context->slavecount << "." << std::endl;
    return false;
  }

  for (int slave = 1; slave <= options_.motor_count; ++slave) {
    if (!configure_slave(slave)) {
      return false;
    }
  }

  const int iomap_bytes = ecx_config_map_group(context, iomap_.data(), 0);
  if (iomap_bytes <= 0) {
    std::cerr << "ec_config_map failed." << std::endl;
    return false;
  }
  std::cout << "IOmap size: " << iomap_bytes << " byte(s)." << std::endl;

  if (options_.use_distributed_clocks) {
    ecx_configdc(context);
    for (int slave = 1; slave <= options_.motor_count; ++slave) {
      ecx_dcsync0(context, slave, TRUE, static_cast<uint32_t>(options_.cycle_time_ns), 0);
    }
  }

  const int expected_wkc = (context->grouplist[0].outputsWKC * 2) + context->grouplist[0].inputsWKC;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    expected_wkc_ = expected_wkc;
  }
  std::cout << "Expected working counter: " << expected_wkc_ << std::endl;

  return enter_operational();
}

void FourMotorMaster::set_runtime_command(
  cia402::Mode mode,
  const std::array<int32_t, kMaxMotors> & targets,
  bool enable_drives,
  uint8_t enable_motor_mask,
  double csp_speed_counts_per_s)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  options_.mode = mode;
  options_.targets = targets;
  options_.enable_drives = enable_drives;
  options_.enable_motor_mask = enable_motor_mask & 0x0f;
  options_.csp_speed_counts_per_s = csp_speed_counts_per_s;
}

void FourMotorMaster::request_fault_reset(uint8_t motor_mask)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  fault_reset_mask_ |= (motor_mask & 0x0f);
}

MasterSnapshot FourMotorMaster::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  MasterSnapshot snap;
  snap.initialized = initialized_;
  snap.in_operational = in_operational_;
  snap.expected_wkc = expected_wkc_;
  snap.last_wkc = last_wkc_;
  snap.mode = options_.mode;
  snap.enable_drives = options_.enable_drives;
  snap.enable_motor_mask = options_.enable_motor_mask;
  snap.targets = options_.targets;
  snap.samples = samples_;
  return snap;
}

bool FourMotorMaster::configure_slave(int slave)
{
  auto * context = &soem_->context;
  auto * slave_info = context->slavelist + slave;
  std::cout << "Configuring slave " << slave << " (" << slave_info->name << ")" << std::endl;

  if (options_.require_expected_identity) {
    if (slave_info->eep_man != options_.expected_vendor_id ||
      slave_info->eep_id != options_.expected_product_id)
    {
      std::cerr << "Slave " << slave << " identity mismatch. Expected vendor/product 0x"
                << std::hex << std::setw(8) << std::setfill('0') << options_.expected_vendor_id
                << "/0x" << std::setw(8) << options_.expected_product_id
                << ", got 0x" << std::setw(8) << slave_info->eep_man
                << "/0x" << std::setw(8) << slave_info->eep_id
                << std::dec << std::setfill(' ') << "." << std::endl;
      return false;
    }
  }

  const int state = ecx_statecheck(context, slave, EC_STATE_PRE_OP, kEcTimeoutState);
  if (state != EC_STATE_PRE_OP) {
    ecx_readstate(context);
    std::cerr << "Slave " << slave << " did not reach PRE_OP before SDO configuration. state=0x"
              << std::hex << slave_info->state << " ALstatus=0x" << slave_info->ALstatuscode
              << " (" << ec_ALstatuscode2string(slave_info->ALstatuscode) << ")"
              << std::dec << std::endl;
    return false;
  }

  return configure_pdo_mapping(slave) && write_startup_sdos(slave);
}

bool FourMotorMaster::configure_pdo_mapping(int slave)
{
  return configure_rxpdo_mapping(slave) && configure_txpdo_mapping(slave);
}

bool FourMotorMaster::configure_rxpdo_mapping(int slave)
{
  (void)slave;
  return true;
}

bool FourMotorMaster::configure_txpdo_mapping(int slave)
{
  const uint8_t zero8 = 0;
  const uint8_t one8 = 1;
  const uint16_t txpdo = 0x1A00;

  return
    sdo_write(slave, 0x1C13, 0, zero8, "clear TxPDO assign") &&
    sdo_write(slave, 0x1C13, 1, txpdo, "assign existing TxPDO map") &&
    sdo_write(slave, 0x1C13, 0, one8, "activate TxPDO assign");
}

bool FourMotorMaster::write_startup_sdos(int slave)
{
  const int8_t interpolation_value = 1;
  const int8_t interpolation_base = -3;
  const uint32_t profile_velocity = 500;
  const uint32_t profile_accel = 5000;
  const uint32_t profile_decel = 5000;

  return
    sdo_write(slave, 0x60C2, 1, interpolation_value, "interpolation time value") &&
    sdo_write(slave, 0x60C2, 2, interpolation_base, "interpolation time base") &&
    sdo_write(slave, 0x6081, 0, profile_velocity, "profile velocity") &&
    sdo_write(slave, 0x6083, 0, profile_accel, "profile acceleration") &&
    sdo_write(slave, 0x6084, 0, profile_decel, "profile deceleration");
}

bool FourMotorMaster::enter_operational()
{
  auto * context = &soem_->context;
  ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  context->slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_send_processdata(context);
  ecx_receive_processdata(context, EC_TIMEOUTRET);
  ecx_writestate(context, 0);

  int checks = 40;
  do {
    ecx_send_processdata(context);
    ecx_receive_processdata(context, EC_TIMEOUTRET);
    ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, 50000);
  } while (checks-- > 0 && context->slavelist[0].state != EC_STATE_OPERATIONAL);

  if (context->slavelist[0].state != EC_STATE_OPERATIONAL) {
    std::cerr << "Not all slaves reached OPERATIONAL." << std::endl;
    ecx_readstate(context);
    for (int slave = 1; slave <= context->slavecount; ++slave) {
      auto * slave_info = context->slavelist + slave;
      if (slave_info->state != EC_STATE_OPERATIONAL) {
        std::cerr << "Slave " << slave << " state=0x" << std::hex << slave_info->state
                  << " ALstatus=0x" << slave_info->ALstatuscode << " ("
                  << ec_ALstatuscode2string(slave_info->ALstatuscode) << ")"
                  << std::dec << std::endl;
      }
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    in_operational_ = true;
  }
  exchange_processdata();
  read_inputs();
  for (int i = 0; i < options_.motor_count; ++i) {
    csp_command_position_[i] = samples_[i].actual_position;
    commands_[i].target_position = samples_[i].actual_position;
  }
  if (options_.hold_current_on_start && mode_is_position(options_.mode)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < options_.motor_count; ++i) {
      options_.targets[i] = samples_[i].actual_position;
    }
  }

  std::cout << "All configured slaves are OPERATIONAL." << std::endl;
  return true;
}

int FourMotorMaster::run(const std::atomic_bool & stop_requested)
{
  if (!in_operational_) {
    std::cerr << "Master is not operational." << std::endl;
    return 1;
  }

  const auto start = std::chrono::steady_clock::now();
  uint64_t cycle = 0;
  timespec next_tick{};
  clock_gettime(CLOCK_MONOTONIC, &next_tick);
  add_ns(next_tick, options_.cycle_time_ns);

  while (!stop_requested.load()) {
    update_commands();
    write_outputs();

    if (!exchange_processdata()) {
      recover_slaves_if_needed();
    }
    read_inputs();

    if (options_.print_every_cycles > 0 && (cycle % options_.print_every_cycles) == 0) {
      print_samples(cycle, last_wkc_);
    }

    ++cycle;
    if (options_.duration_s > 0.0) {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= options_.duration_s) {
        break;
      }
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);
    add_ns(next_tick, options_.cycle_time_ns);
  }

  for (int i = 0; i < options_.motor_count; ++i) {
    commands_[i].control_word = cia402::control_word::disable_voltage;
    commands_[i].target_velocity = 0;
  }
  write_outputs();
  exchange_processdata();

  return 0;
}

void FourMotorMaster::shutdown()
{
  if (!initialized_) {
    return;
  }

  if (in_operational_) {
    for (int i = 0; i < options_.motor_count; ++i) {
      commands_[i].control_word = cia402::control_word::disable_voltage;
      commands_[i].target_velocity = 0;
    }
    write_outputs();
    exchange_processdata();

    auto * context = &soem_->context;
    context->slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(context, 0);
    ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  }

  ecx_close(&soem_->context);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = false;
    in_operational_ = false;
  }
}

bool FourMotorMaster::exchange_processdata()
{
  auto * context = &soem_->context;
  ecx_send_processdata(context);
  const int wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_wkc_ = wkc;
  }
  return wkc >= expected_wkc_;
}

void FourMotorMaster::read_inputs()
{
  for (int i = 0; i < options_.motor_count; ++i) {
    const int slave = i + 1;
    TxPdo pdo{};
    auto * slave_info = soem_->context.slavelist + slave;
    if (slave_info->inputs != nullptr) {
      std::memcpy(&pdo, slave_info->inputs, sizeof(pdo));
    }

    MotorSample sample;
    sample.status_word = pdo.status_word;
    sample.actual_position = pdo.actual_position;
    sample.actual_velocity = pdo.actual_velocity;
    sample.mode_display = pdo.mode_display;
    sample.physical_position_mm = physical_position_mm(i, pdo.actual_position);
    sample.physical_velocity_mm_s = physical_velocity_mm_s(i, pdo.actual_velocity);

    std::lock_guard<std::mutex> lock(state_mutex_);
    samples_[i] = sample;
  }
}

void FourMotorMaster::write_outputs()
{
  for (int i = 0; i < options_.motor_count; ++i) {
    const int slave = i + 1;
    RxPdo pdo{};
    pdo.control_word = commands_[i].control_word;
    pdo.target_position = commands_[i].target_position;
    pdo.target_velocity = commands_[i].target_velocity;
    pdo.mode_of_operation = commands_[i].mode_of_operation;

    auto * slave_info = soem_->context.slavelist + slave;
    if (slave_info->outputs != nullptr) {
      std::memcpy(slave_info->outputs, &pdo, sizeof(pdo));
    }
  }
}

void FourMotorMaster::update_commands()
{
  cia402::Mode mode;
  std::array<int32_t, kMaxMotors> targets{};
  bool enable_drives = false;
  uint8_t enable_motor_mask = 0;
  uint8_t fault_reset_mask = 0;
  double csp_speed_counts_per_s = 0.0;
  std::array<MotorSample, kMaxMotors> samples{};

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mode = options_.mode;
    targets = options_.targets;
    enable_drives = options_.enable_drives;
    enable_motor_mask = options_.enable_motor_mask;
    csp_speed_counts_per_s = options_.csp_speed_counts_per_s;
    fault_reset_mask = fault_reset_mask_;
    fault_reset_mask_ = 0;
    samples = samples_;
  }

  const int8_t mode_code = static_cast<int8_t>(mode);
  const double step =
    csp_speed_counts_per_s <= 0.0 ? 0.0 :
    csp_speed_counts_per_s * static_cast<double>(options_.cycle_time_ns) / 1.0e9;

  for (int i = 0; i < options_.motor_count; ++i) {
    const uint8_t motor_bit = static_cast<uint8_t>(1u << i);
    const bool motor_enabled = (enable_motor_mask & motor_bit) != 0;
    commands_[i].mode_of_operation = motor_enabled ? mode_code : static_cast<int8_t>(cia402::Mode::NoMode);

    if ((fault_reset_mask & motor_bit) != 0) {
      commands_[i].control_word = cia402::control_word::fault_reset;
    } else if (enable_drives && motor_enabled) {
      commands_[i].control_word =
        cia402::next_enable_control_word(samples[i].status_word, commands_[i].control_word);
    } else {
      commands_[i].control_word = cia402::control_word::disable_voltage;
    }

    if (mode_is_position(mode)) {
      if (mode == cia402::Mode::Csp && step > 0.0) {
        const double target = static_cast<double>(targets[i]);
        const double delta = target - csp_command_position_[i];
        if (std::abs(delta) <= step) {
          csp_command_position_[i] = target;
        } else {
          csp_command_position_[i] += std::copysign(step, delta);
        }
        commands_[i].target_position = static_cast<int32_t>(std::llround(csp_command_position_[i]));
      } else {
        commands_[i].target_position = targets[i];
      }
      commands_[i].target_velocity = 0;
    } else if (mode_is_velocity(mode)) {
      commands_[i].target_velocity = targets[i];
      commands_[i].target_position = samples[i].actual_position;
    } else {
      commands_[i].target_velocity = 0;
      commands_[i].target_position = samples[i].actual_position;
    }
  }
}

void FourMotorMaster::recover_slaves_if_needed()
{
  auto * context = &soem_->context;
  auto * group = context->grouplist + 0;
  if (last_wkc_ >= expected_wkc_ && !group->docheckstate) {
    return;
  }

  group->docheckstate = FALSE;
  ecx_readstate(context);

  for (int slave = 1; slave <= options_.motor_count; ++slave) {
    auto * slave_info = context->slavelist + slave;
    if (slave_info->state == EC_STATE_OPERATIONAL) {
      continue;
    }

    group->docheckstate = TRUE;
    if (slave_info->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
      std::cerr << "Slave " << slave << " SAFE_OP + ERROR, ALstatus=0x"
                << std::hex << slave_info->ALstatuscode << " ("
                << ec_ALstatuscode2string(slave_info->ALstatuscode) << ")"
                << std::dec << ", acknowledging." << std::endl;
      slave_info->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
      ecx_writestate(context, slave);
    } else if (slave_info->state == EC_STATE_SAFE_OP) {
      std::cerr << "Slave " << slave << " SAFE_OP, requesting OP." << std::endl;
      slave_info->state = EC_STATE_OPERATIONAL;
      ecx_writestate(context, slave);
    } else if (slave_info->state > EC_STATE_NONE) {
      if (ecx_reconfig_slave(context, slave, kEcTimeoutMon)) {
        slave_info->islost = FALSE;
        std::cerr << "Slave " << slave << " reconfigured." << std::endl;
      }
    } else if (!slave_info->islost) {
      ecx_statecheck(context, slave, EC_STATE_OPERATIONAL, kEcTimeoutMon);
      if (slave_info->state == EC_STATE_NONE) {
        slave_info->islost = TRUE;
        std::cerr << "Slave " << slave << " lost." << std::endl;
      }
    }

    if (slave_info->islost) {
      if (slave_info->state == EC_STATE_NONE) {
        if (ecx_recover_slave(context, slave, kEcTimeoutMon)) {
          slave_info->islost = FALSE;
          std::cerr << "Slave " << slave << " recovered." << std::endl;
        }
      } else {
        slave_info->islost = FALSE;
        std::cerr << "Slave " << slave << " found again." << std::endl;
      }
    }
  }
}

void FourMotorMaster::print_samples(uint64_t cycle, int wkc) const
{
  std::cout << "cycle=" << cycle << " wkc=" << wkc << "/" << expected_wkc_
            << " mode=" << cia402::to_string(options_.mode)
            << " enable_mask=0x" << std::hex << static_cast<int>(options_.enable_motor_mask)
            << std::dec << std::endl;
  for (int i = 0; i < options_.motor_count; ++i) {
    const auto & s = samples_[i];
    const auto & c = commands_[i];
    std::cout << "  M" << (i + 1)
              << " sw=0x" << std::hex << std::setw(4) << std::setfill('0') << s.status_word
              << std::dec << std::setfill(' ')
              << " state=\"" << cia402::to_string(cia402::decode_status_word(s.status_word)) << "\""
              << " cw=0x" << std::hex << std::setw(4) << std::setfill('0') << c.control_word
              << std::dec << std::setfill(' ')
              << " pos=" << s.actual_position
              << " vel=" << s.actual_velocity
              << " op=" << static_cast<int>(s.mode_display)
              << " cmd_op=" << static_cast<int>(c.mode_of_operation)
              << " tgt_pos=" << c.target_position
              << " tgt_vel=" << c.target_velocity
              << " phys_pos_mm=" << std::fixed << std::setprecision(3) << s.physical_position_mm
              << " phys_vel_mm_s=" << s.physical_velocity_mm_s
              << std::defaultfloat << std::endl;
  }
}

double FourMotorMaster::physical_position_mm(int motor_index, int32_t raw_position) const
{
  const auto & scale = options_.scales[motor_index];
  if (scale.encoder_counts_per_rev == 0) {
    return 0.0;
  }
  return (static_cast<double>(scale.position_bias_count) - static_cast<double>(raw_position)) *
    scale.screw_lead_mm_per_rev / static_cast<double>(scale.encoder_counts_per_rev);
}

double FourMotorMaster::physical_velocity_mm_s(int motor_index, int32_t raw_velocity) const
{
  const auto & scale = options_.scales[motor_index];
  return -static_cast<double>(raw_velocity) * scale.screw_lead_mm_per_rev / 1000.0;
}

template<typename T>
bool FourMotorMaster::sdo_write(int slave, uint16_t index, uint8_t sub_index, T value, const char * label)
{
  int size = static_cast<int>(sizeof(T));
  const int result = ecx_SDOwrite(&soem_->context, slave, index, sub_index, FALSE, size, &value, EC_TIMEOUTRXM);
  if (result <= 0) {
    std::cerr << "SDO write failed on slave " << slave << " for " << label
              << " (0x" << std::hex << index << ":" << static_cast<int>(sub_index)
              << std::dec << ", result=" << result << ")." << std::endl;
    print_soem_errors();
    return false;
  }
  return true;
}

void FourMotorMaster::print_soem_errors()
{
  bool had_error = false;
  while (soem_->context.ecaterror) {
    had_error = true;
    std::cerr << ecx_elist2string(&soem_->context);
  }
  if (!had_error) {
    std::cerr << "No SDO abort detail was returned by the slave; this is usually a mailbox timeout "
              << "or no valid SDO response." << std::endl;
  }
}

template bool FourMotorMaster::sdo_write<uint8_t>(int, uint16_t, uint8_t, uint8_t, const char *);
template bool FourMotorMaster::sdo_write<int8_t>(int, uint16_t, uint8_t, int8_t, const char *);
template bool FourMotorMaster::sdo_write<uint16_t>(int, uint16_t, uint8_t, uint16_t, const char *);
template bool FourMotorMaster::sdo_write<uint32_t>(int, uint16_t, uint8_t, uint32_t, const char *);

}  // namespace mpk_soem
