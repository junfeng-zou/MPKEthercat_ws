#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mpk_soem/cia402.hpp"

namespace mpk_soem
{

constexpr int kMaxMotors = 4;

struct MotorScale
{
  int32_t position_bias_count{0};
  int32_t encoder_counts_per_rev{131072};
  double screw_lead_mm_per_rev{10.0};
};

struct MotorSample
{
  uint16_t status_word{0};
  int32_t actual_position{0};
  int32_t actual_velocity{0};
  int8_t mode_display{0};
  double physical_position_mm{0.0};
  double physical_velocity_mm_s{0.0};
};

struct MotorCommand
{
  uint16_t control_word{0};
  int32_t target_position{0};
  int32_t target_velocity{0};
  int8_t mode_of_operation{0};
};

struct MasterOptions
{
  std::string ifname;
  int motor_count{kMaxMotors};
  cia402::Mode mode{cia402::Mode::Csp};
  std::array<int32_t, kMaxMotors> targets{0, 0, 0, 0};
  std::array<MotorScale, kMaxMotors> scales{};
  bool enable_drives{false};
  bool use_distributed_clocks{true};
  bool require_expected_identity{true};
  uint32_t expected_vendor_id{0x0000029c};
  uint32_t expected_product_id{0x03831002};
  int64_t cycle_time_ns{1000000};
  double duration_s{0.0};
  double csp_speed_counts_per_s{0.0};
  int print_every_cycles{1000};
  uint8_t enable_motor_mask{0x0f};
  bool hold_current_on_start{false};
};

struct MasterSnapshot
{
  bool initialized{false};
  bool in_operational{false};
  int expected_wkc{0};
  int last_wkc{0};
  cia402::Mode mode{cia402::Mode::NoMode};
  bool enable_drives{false};
  uint8_t enable_motor_mask{0};
  std::array<int32_t, kMaxMotors> targets{0, 0, 0, 0};
  std::array<MotorSample, kMaxMotors> samples{};
};

class FourMotorMaster
{
public:
  explicit FourMotorMaster(MasterOptions options);
  ~FourMotorMaster();

  FourMotorMaster(const FourMotorMaster &) = delete;
  FourMotorMaster & operator=(const FourMotorMaster &) = delete;

  bool initialize();
  int run(const std::atomic_bool & stop_requested);
  void shutdown();
  void set_runtime_command(
    cia402::Mode mode,
    const std::array<int32_t, kMaxMotors> & targets,
    bool enable_drives,
    uint8_t enable_motor_mask,
    double csp_speed_counts_per_s);
  void request_fault_reset(uint8_t motor_mask);
  MasterSnapshot snapshot() const;

private:
  bool configure_slave(int slave);
  bool configure_pdo_mapping(int slave);
  bool configure_rxpdo_mapping(int slave);
  bool configure_txpdo_mapping(int slave);
  bool write_startup_sdos(int slave);
  bool enter_operational();
  bool exchange_processdata();
  void read_inputs();
  void write_outputs();
  void update_commands();
  void recover_slaves_if_needed();
  void print_samples(uint64_t cycle, int wkc) const;

  double physical_position_mm(int motor_index, int32_t raw_position) const;
  double physical_velocity_mm_s(int motor_index, int32_t raw_velocity) const;

  template<typename T>
  bool sdo_write(int slave, uint16_t index, uint8_t sub_index, T value, const char * label);
  void print_soem_errors();

  struct SoemState;

  MasterOptions options_;
  std::unique_ptr<SoemState> soem_;
  std::vector<uint8_t> iomap_;
  int expected_wkc_{0};
  int last_wkc_{0};
  bool initialized_{false};
  bool in_operational_{false};
  std::array<MotorSample, kMaxMotors> samples_{};
  std::array<MotorCommand, kMaxMotors> commands_{};
  std::array<double, kMaxMotors> csp_command_position_{};
  mutable std::mutex state_mutex_;
  uint8_t fault_reset_mask_{0};
};

}  // namespace mpk_soem
