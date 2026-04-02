#include "motor_data_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace MPKEthercat_test
{

MotorDataBroadcaster::MotorDataBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MotorDataBroadcaster::on_init()
{
  if (!get_node()->has_parameter("position_bias_count")) {
    get_node()->declare_parameter<int>("position_bias_count", 0);
  }
  if (!get_node()->has_parameter("encoder_counts_per_rev")) {
    get_node()->declare_parameter<int>("encoder_counts_per_rev", 131072);
  }
  if (!get_node()->has_parameter("screw_lead_mm_per_rev")) {
    get_node()->declare_parameter<double>("screw_lead_mm_per_rev", 10.0);
  }

  publisher_ = get_node()->create_publisher<mpk_ethercat_test::msg::MotorData>(
    "~/motor_data_topic", rclcpp::SystemDefaultsQoS());
  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&MotorDataBroadcaster::on_set_parameters, this, std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorDataBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto bias = static_cast<int32_t>(get_node()->get_parameter("position_bias_count").as_int());
  const auto counts = static_cast<int32_t>(get_node()->get_parameter("encoder_counts_per_rev").as_int());
  const auto lead = get_node()->get_parameter("screw_lead_mm_per_rev").as_double();
  position_bias_count_.store(bias);
  encoder_counts_per_rev_.store(counts);
  screw_lead_mm_per_rev_.store(lead);
  if (encoder_counts_per_rev_.load() <= 0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "encoder_counts_per_rev <= 0 (%d), fallback to 131072",
      encoder_counts_per_rev_.load());
    encoder_counts_per_rev_.store(131072);
  }
  if (!std::isfinite(screw_lead_mm_per_rev_.load()) || screw_lead_mm_per_rev_.load() == 0.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "screw_lead_mm_per_rev invalid (%f), fallback to 10.0",
      screw_lead_mm_per_rev_.load());
    screw_lead_mm_per_rev_.store(10.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Distance conversion enabled: distance_mm=(bias-pos)*lead/counts, bias=%d, lead=%.6f, counts=%d",
    position_bias_count_.load(),
    screw_lead_mm_per_rev_.load(),
    encoder_counts_per_rev_.load());
  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult MotorDataBroadcaster::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";

  for (const auto & p : parameters) {
    if (p.get_name() == "position_bias_count") {
      position_bias_count_.store(static_cast<int32_t>(p.as_int()));
    } else if (p.get_name() == "encoder_counts_per_rev") {
      const auto v = static_cast<int32_t>(p.as_int());
      if (v <= 0) {
        result.successful = false;
        result.reason = "encoder_counts_per_rev must > 0";
        return result;
      }
      encoder_counts_per_rev_.store(v);
    } else if (p.get_name() == "screw_lead_mm_per_rev") {
      const auto v = p.as_double();
      if (!std::isfinite(v) || v == 0.0) {
        result.successful = false;
        result.reason = "screw_lead_mm_per_rev must be finite and non-zero";
        return result;
      }
      screw_lead_mm_per_rev_.store(v);
    }
  }
  return result;
}

controller_interface::CallbackReturn MotorDataBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (publisher_) {
    publisher_->on_activate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorDataBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorDataBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration 
MotorDataBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // 按照 URDF 中的 关节名/接口名 申请
  config.names.push_back("motor_joint_1/status_word");
  config.names.push_back("motor_joint_1/position");
  config.names.push_back("motor_joint_1/velocity");
  config.names.push_back("motor_joint_1/op_mode");
  
  return config;
}


controller_interface::return_type MotorDataBroadcaster::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    {
    auto msg = mpk_ethercat_test::msg::MotorData();
    msg.header.stamp = time;

    // 注意：这里的索引 0,1,2,3 对应 state_interface_configuration 的 push_back 顺序
    const double sw_raw = state_interfaces_[0].get_value();
    const double pos_raw = state_interfaces_[1].get_value();
    const double vel_raw = state_interfaces_[2].get_value();
    const double op_raw = state_interfaces_[3].get_value();

    const bool valid =
      std::isfinite(sw_raw) && std::isfinite(pos_raw) &&
      std::isfinite(vel_raw) && std::isfinite(op_raw);

    if (valid) {
      msg.status_word = static_cast<uint16_t>(sw_raw);
      msg.actual_position = static_cast<int32_t>(pos_raw);
      msg.actual_velocity = static_cast<int32_t>(vel_raw);
      msg.operation_mode = static_cast<int8_t>(op_raw);
      const auto bias = position_bias_count_.load();
      const auto lead = screw_lead_mm_per_rev_.load();
      const auto counts = encoder_counts_per_rev_.load();
      msg.actual_physical_position =
        (static_cast<double>(bias) - static_cast<double>(msg.actual_position)) *
        lead / static_cast<double>(counts);
      // 0x606C 实际速度单位按 mrev/s 处理：mm/s = mrev/s * lead(mm/rev) / 1000
      // 方向定义与位置一致：编码器速度正方向对应物理负方向
      msg.actual_physical_velocity =
        -static_cast<double>(msg.actual_velocity) * lead / 1000.0;

      last_status_word_ = msg.status_word;
      last_position_ = msg.actual_position;
      last_velocity_ = msg.actual_velocity;
      last_operation_mode_ = msg.operation_mode;
      last_actual_physical_position_ = msg.actual_physical_position;
      last_actual_physical_velocity_ = msg.actual_physical_velocity;
      has_last_sample_ = true;
    } else if (has_last_sample_) {
      // 通信抖动/状态无效时保留上次有效值，避免 NaN 强转后出现“全 0”假象
      msg.status_word = last_status_word_;
      msg.actual_position = last_position_;
      msg.actual_velocity = last_velocity_;
      msg.operation_mode = last_operation_mode_;
      msg.actual_physical_position = last_actual_physical_position_;
      msg.actual_physical_velocity = last_actual_physical_velocity_;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *(get_node()->get_clock()),
        2000,
        "Invalid motor state sample (NaN/non-finite), publish last valid sample.");
    } else {
      // 启动早期尚无有效样本时，发布零值并给出提示
      msg.status_word = 0;
      msg.actual_position = 0;
      msg.actual_velocity = 0;
      msg.operation_mode = 0;
      msg.actual_physical_position = 0.0;
      msg.actual_physical_velocity = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *(get_node()->get_clock()),
        2000,
        "No valid motor state sample yet, publish zero sample.");
    }

    publisher_->publish(msg);

    return controller_interface::return_type::OK;
    }

}  // namespace MPKEthercat_test

PLUGINLIB_EXPORT_CLASS(
  MPKEthercat_test::MotorDataBroadcaster,
  controller_interface::ControllerInterface)
