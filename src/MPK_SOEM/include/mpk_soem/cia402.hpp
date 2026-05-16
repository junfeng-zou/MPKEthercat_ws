#pragma once

#include <cstdint>
#include <string>

namespace mpk_soem::cia402
{

enum class DriveState
{
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  Unknown
};

enum class Mode : int8_t
{
  NoMode = 0,
  ProfilePosition = 1,
  ProfileVelocity = 3,
  Homing = 6,
  InterpolatedPosition = 7,
  Csp = 8,
  Csv = 9,
  Cst = 10
};

namespace control_word
{
constexpr uint16_t disable_voltage = 0x0000;
constexpr uint16_t quick_stop = 0x0002;
constexpr uint16_t shutdown = 0x0006;
constexpr uint16_t switch_on = 0x0007;
constexpr uint16_t enable_operation = 0x000F;
constexpr uint16_t fault_reset = 0x0080;
}  // namespace control_word

DriveState decode_status_word(uint16_t status_word);
const char * to_string(DriveState state);
const char * to_string(Mode mode);

bool is_operation_enabled(uint16_t status_word);
bool is_fault(uint16_t status_word);

uint16_t next_enable_control_word(uint16_t status_word, uint16_t previous_control_word);
Mode parse_mode(const std::string & mode_name);

}  // namespace mpk_soem::cia402
