#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpk_ethercat_test/msg/motor_data.hpp"

using std::placeholders::_1;

struct RunnerArgs
{
  std::string mode;
  double rate_hz{500.0};
  std::optional<double> target;
  std::optional<double> csp_start;
  std::optional<double> csp_step_per_cycle;
  std::optional<double> csp_speed;
  std::string csp_dynamic_target_topic;
  bool require_enabled{false};
  bool require_mode{false};
  bool auto_enable{false};
  std::string topic{"/MotorDataBroadcaster/motor_data_topic"};
  std::string control_word_topic{"/control_word_controller/commands"};
  std::string op_mode_topic{"/op_mode_controller/commands"};
  std::string position_topic{"/position_controller/commands"};
  std::string velocity_topic{"/velocity_controller/commands"};
};

static std::optional<double> parse_double(const std::string & s)
{
  char * endptr = nullptr;
  const double v = std::strtod(s.c_str(), &endptr);
  if (endptr == s.c_str() || (endptr != nullptr && *endptr != '\0')) {
    return std::nullopt;
  }
  return v;
}

static bool parse_args(int argc, char ** argv, RunnerArgs & args, std::string & err)
{
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next_value = [&](const std::string & key) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        err = "missing value for " + key;
        return std::nullopt;
      }
      ++i;
      return std::string(argv[i]);
    };

    if (a == "--mode") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.mode = *v;
    } else if (a == "--rate-hz") {
      auto v = next_value(a);
      if (!v) {return false;}
      auto d = parse_double(*v);
      if (!d) {err = "invalid --rate-hz"; return false;}
      args.rate_hz = *d;
    } else if (a == "--target") {
      auto v = next_value(a);
      if (!v) {return false;}
      auto d = parse_double(*v);
      if (!d) {err = "invalid --target"; return false;}
      args.target = *d;
    } else if (a == "--csp-start") {
      auto v = next_value(a);
      if (!v) {return false;}
      auto d = parse_double(*v);
      if (!d) {err = "invalid --csp-start"; return false;}
      args.csp_start = *d;
    } else if (a == "--csp-step-per-cycle") {
      auto v = next_value(a);
      if (!v) {return false;}
      auto d = parse_double(*v);
      if (!d) {err = "invalid --csp-step-per-cycle"; return false;}
      args.csp_step_per_cycle = *d;
    } else if (a == "--csp-speed") {
      auto v = next_value(a);
      if (!v) {return false;}
      auto d = parse_double(*v);
      if (!d) {err = "invalid --csp-speed"; return false;}
      args.csp_speed = *d;
    } else if (a == "--csp-dynamic-target-topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.csp_dynamic_target_topic = *v;
    } else if (a == "--require-enabled") {
      args.require_enabled = true;
    } else if (a == "--require-mode") {
      args.require_mode = true;
    } else if (a == "--auto-enable") {
      args.auto_enable = true;
    } else if (a == "--topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.topic = *v;
    } else if (a == "--control-word-topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.control_word_topic = *v;
    } else if (a == "--op-mode-topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.op_mode_topic = *v;
    } else if (a == "--position-topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.position_topic = *v;
    } else if (a == "--velocity-topic") {
      auto v = next_value(a);
      if (!v) {return false;}
      args.velocity_topic = *v;
    } else {
      err = "unknown argument: " + a;
      return false;
    }
  }

  if (args.mode != "csp" && args.mode != "csv") {
    err = "--mode must be csp or csv";
    return false;
  }
  if (args.rate_hz <= 0.0) {
    err = "--rate-hz must > 0";
    return false;
  }
  return true;
}

static std::string state_machine_str(uint16_t sw)
{
  const uint16_t v_6f = sw & 0x6F;
  const uint16_t v_4f = sw & 0x4F;
  if (v_4f == 0x00) {return "Not ready to switch on";}
  if (v_4f == 0x40) {return "Switch on disabled";}
  if (v_6f == 0x21) {return "Ready to switch on";}
  if (v_6f == 0x23) {return "Switched on";}
  if (v_6f == 0x27) {return "Operation enabled";}
  if (v_6f == 0x07) {return "Quick stop active";}
  if (v_4f == 0x0F) {return "Fault reaction active";}
  if (v_4f == 0x08) {return "Fault";}
  return "Unknown";
}

class CyclicModeRunner : public rclcpp::Node
{
public:
  explicit CyclicModeRunner(const RunnerArgs & args)
  : Node("cyclic_mode_runner"), args_(args)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    sub_ = create_subscription<mpk_ethercat_test::msg::MotorData>(
      args_.topic, qos, std::bind(&CyclicModeRunner::on_motor_data, this, _1));
    pub_op_mode_ = create_publisher<std_msgs::msg::Float64MultiArray>(args_.op_mode_topic, 10);
    pub_control_word_ = create_publisher<std_msgs::msg::Float64MultiArray>(args_.control_word_topic, 10);
    pub_position_ = create_publisher<std_msgs::msg::Float64MultiArray>(args_.position_topic, 10);
    pub_velocity_ = create_publisher<std_msgs::msg::Float64MultiArray>(args_.velocity_topic, 10);
    if (args_.mode == "csp" && !args_.csp_dynamic_target_topic.empty()) {
      sub_dynamic_target_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        args_.csp_dynamic_target_topic, 10, std::bind(&CyclicModeRunner::on_dynamic_target, this, _1));
    }

    mode_code_ = (args_.mode == "csp") ? 8 : 9;
    RCLCPP_INFO(
      get_logger(), "Runner started: mode=%s(%d), rate=%.3fHz",
      args_.mode.c_str(), mode_code_, args_.rate_hz);
  }

  void run(rclcpp::executors::SingleThreadedExecutor & executor)
  {
    const double period_s = 1.0 / args_.rate_hz;
    auto next_due = std::chrono::steady_clock::now() + std::chrono::duration<double>(period_s);
    uint64_t sent = 0;
    const uint64_t report_every = std::max<uint64_t>(1, static_cast<uint64_t>(std::llround(args_.rate_hz)));
    bool wait_logged = false;

    while (rclcpp::ok()) {
      try {
        executor.spin_some(std::chrono::milliseconds(0));
      } catch (const std::exception & e) {
        const std::string what = e.what();
        if (what.find("Unable to convert call argument to Python object") != std::string::npos) {
          const auto now = std::chrono::steady_clock::now();
          if ((now - last_spin_err_tp_) >= std::chrono::seconds(1)) {
            safe_warn("Transient subscription decode error, skipping this cycle.");
            last_spin_err_tp_ = now;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        if (what.find("context is not valid") != std::string::npos) {
          safe_info("ROS context became invalid, exiting runner loop...");
          break;
        }
        throw;
      }

      periodic_mode_and_enable();

      if (args_.require_enabled && !state_ok()) {
        if (!wait_logged) {
          RCLCPP_INFO(get_logger(), "Waiting for Operation enabled...");
          wait_logged = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (args_.require_mode && !mode_ok()) {
        if (!wait_logged) {
          RCLCPP_INFO(get_logger(), "Waiting for 0x6061 == %d ...", mode_code_);
          wait_logged = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      wait_logged = false;

      const auto now = std::chrono::steady_clock::now();
      if (now < next_due) {
        auto remain = next_due - now;
        if (remain > std::chrono::milliseconds(1)) {
          remain = std::chrono::milliseconds(1);
        }
        std::this_thread::sleep_for(remain);
        continue;
      }

      const int cmd = next_cmd(period_s);
      if (args_.mode == "csp") {
        pub_position_->publish(msg1(static_cast<double>(cmd)));
      } else {
        pub_velocity_->publish(msg1(static_cast<double>(cmd)));
      }
      ++sent;
      if (sent % report_every == 0) {
        RCLCPP_INFO(get_logger(), "sent=%lu, last_cmd=%d", sent, cmd);
      }

      const auto now2 = std::chrono::steady_clock::now();
      while (next_due <= now2) {
        next_due += std::chrono::duration<double>(period_s);
      }
    }
  }

private:
  static std_msgs::msg::Float64MultiArray msg1(double v)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(v);
    return msg;
  }

  void safe_info(const std::string & text) const
  {
    if (rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
      return;
    }
    std::cout << "[cyclic_mode_runner] " << text << std::endl;
  }

  void safe_warn(const std::string & text) const
  {
    if (rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
      return;
    }
    std::cout << "[cyclic_mode_runner][WARN] " << text << std::endl;
  }

  void on_motor_data(const mpk_ethercat_test::msg::MotorData::SharedPtr msg)
  {
    latest_status_word_ = msg->status_word;
    latest_op_mode_ = msg->operation_mode;
    latest_position_ = static_cast<double>(msg->actual_position);
  }

  void on_dynamic_target(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    latest_dynamic_target_ = static_cast<double>(msg->data[0]);
  }

  bool state_ok() const
  {
    if (!latest_status_word_) {return false;}
    return state_machine_str(*latest_status_word_) == "Operation enabled";
  }

  bool mode_ok() const
  {
    if (!latest_op_mode_) {return false;}
    return static_cast<int>(*latest_op_mode_) == mode_code_;
  }

  void periodic_mode_and_enable()
  {
    const auto now = std::chrono::steady_clock::now();
    if ((now - last_op_mode_tp_) >= std::chrono::milliseconds(200)) {
      pub_op_mode_->publish(msg1(static_cast<double>(mode_code_)));
      last_op_mode_tp_ = now;
    }
    if (!args_.auto_enable || !latest_status_word_) {return;}
    const auto state = state_machine_str(*latest_status_word_);
    if (state == "Switched on" && (now - last_enable_tp_) >= std::chrono::milliseconds(50)) {
      pub_control_word_->publish(msg1(0x000F));
      last_enable_tp_ = now;
    }
  }

  int next_cmd(double period_s)
  {
    if (args_.mode == "csp") {
      if (!cmd_value_) {
        if (args_.csp_start) {
          cmd_value_ = *args_.csp_start;
        } else if (latest_position_) {
          cmd_value_ = *latest_position_;
        } else {
          cmd_value_ = 0.0;
        }
      }

      std::optional<double> effective_target = latest_dynamic_target_;
      if (!effective_target && args_.target) {
        effective_target = args_.target;
      }

      if (effective_target && args_.csp_speed) {
        const double target = *effective_target;
        const double step_max = std::abs(*args_.csp_speed) * period_s;
        const double delta = target - *cmd_value_;
        if (delta > step_max) {
          *cmd_value_ += step_max;
        } else if (delta < -step_max) {
          *cmd_value_ -= step_max;
        } else {
          *cmd_value_ = target;
        }
      } else if (args_.csp_step_per_cycle) {
        *cmd_value_ += *args_.csp_step_per_cycle;
      } else if (args_.csp_speed) {
        *cmd_value_ += *args_.csp_speed * period_s;
      } else if (effective_target) {
        *cmd_value_ = *effective_target;
      }
      return static_cast<int>(std::llround(*cmd_value_));
    }

    if (args_.target) {
      return static_cast<int>(std::llround(*args_.target));
    }
    return 0;
  }

  RunnerArgs args_;
  int mode_code_{8};
  std::optional<uint16_t> latest_status_word_;
  std::optional<int8_t> latest_op_mode_;
  std::optional<double> latest_position_;
  std::optional<double> latest_dynamic_target_;
  std::optional<double> cmd_value_;
  std::chrono::steady_clock::time_point last_op_mode_tp_{std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point last_enable_tp_{std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point last_spin_err_tp_{std::chrono::steady_clock::time_point::min()};

  rclcpp::Subscription<mpk_ethercat_test::msg::MotorData>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_dynamic_target_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_op_mode_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_control_word_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_position_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_velocity_;
};

int main(int argc, char ** argv)
{
  RunnerArgs args;
  std::string err;
  if (!parse_args(argc, argv, args, err)) {
    std::cerr << "cyclic_mode_runner argument error: " << err << std::endl;
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CyclicModeRunner>(args);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  try {
    node->run(executor);
  } catch (const std::exception & e) {
    std::cerr << "cyclic_mode_runner fatal error: " << e.what() << std::endl;
  }

  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
