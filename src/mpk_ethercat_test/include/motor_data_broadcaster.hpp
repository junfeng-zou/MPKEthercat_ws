#ifndef MPKETHERCAT_TEST__MOTOR_DATA_BROADCASTER_HPP_
#define MPKETHERCAT_TEST__MOTOR_DATA_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "mpk_ethercat_test/msg/motor_data.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <atomic>

namespace MPKEthercat_test
{

class MotorDataBroadcaster : public controller_interface::ControllerInterface
{
public:
  MotorDataBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp_lifecycle::LifecyclePublisher<mpk_ethercat_test::msg::MotorData>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  bool has_last_sample_{false};
  uint16_t last_status_word_{0};
  int32_t last_position_{0};
  int32_t last_velocity_{0};
  int8_t last_operation_mode_{0};
  double last_actual_physical_position_{0.0};
  double last_actual_physical_velocity_{0.0};

  // 物理量换算参数：distance_mm = (position_bias_count - actual_position) * lead_mm / counts_per_rev
  std::atomic<int32_t> position_bias_count_{0};
  std::atomic<int32_t> encoder_counts_per_rev_{131072};  // 17-bit single-turn encoder
  std::atomic<double> screw_lead_mm_per_rev_{10.0};
};

}  // namespace MPKEthercat_test

#endif  // MPKETHERCAT_TEST__MOTOR_DATA_BROADCASTER_HPP_
