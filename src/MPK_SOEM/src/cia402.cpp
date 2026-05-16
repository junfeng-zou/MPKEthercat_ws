#include "mpk_soem/cia402.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace mpk_soem::cia402
{

DriveState decode_status_word(uint16_t status_word)
{
  const uint16_t masked_4f = status_word & 0x004F;
  const uint16_t masked_6f = status_word & 0x006F;

  if (masked_4f == 0x0000) {
    return DriveState::NotReadyToSwitchOn;
  }
  if (masked_4f == 0x0040) {
    return DriveState::SwitchOnDisabled;
  }
  if (masked_6f == 0x0021) {
    return DriveState::ReadyToSwitchOn;
  }
  if (masked_6f == 0x0023) {
    return DriveState::SwitchedOn;
  }
  if (masked_6f == 0x0027) {
    return DriveState::OperationEnabled;
  }
  if (masked_6f == 0x0007) {
    return DriveState::QuickStopActive;
  }
  if (masked_4f == 0x000F) {
    return DriveState::FaultReactionActive;
  }
  if (masked_4f == 0x0008) {
    return DriveState::Fault;
  }
  return DriveState::Unknown;
}

const char * to_string(DriveState state)
{
  switch (state) {
    case DriveState::NotReadyToSwitchOn:
      return "Not ready to switch on";
    case DriveState::SwitchOnDisabled:
      return "Switch on disabled";
    case DriveState::ReadyToSwitchOn:
      return "Ready to switch on";
    case DriveState::SwitchedOn:
      return "Switched on";
    case DriveState::OperationEnabled:
      return "Operation enabled";
    case DriveState::QuickStopActive:
      return "Quick stop active";
    case DriveState::FaultReactionActive:
      return "Fault reaction active";
    case DriveState::Fault:
      return "Fault";
    case DriveState::Unknown:
    default:
      return "Unknown";
  }
}

const char * to_string(Mode mode)
{
  switch (mode) {
    case Mode::NoMode:
      return "No mode";
    case Mode::ProfilePosition:
      return "Profile position";
    case Mode::ProfileVelocity:
      return "Profile velocity";
    case Mode::Homing:
      return "Homing";
    case Mode::InterpolatedPosition:
      return "Interpolated position";
    case Mode::Csp:
      return "CSP";
    case Mode::Csv:
      return "CSV";
    case Mode::Cst:
      return "CST";
    default:
      return "Unknown";
  }
}

bool is_operation_enabled(uint16_t status_word)
{
  return decode_status_word(status_word) == DriveState::OperationEnabled;
}

bool is_fault(uint16_t status_word)
{
  const auto state = decode_status_word(status_word);
  return state == DriveState::Fault || state == DriveState::FaultReactionActive;
}

uint16_t next_enable_control_word(uint16_t status_word, uint16_t previous_control_word)
{
  switch (decode_status_word(status_word)) {
    case DriveState::Fault:
      return previous_control_word == control_word::fault_reset ?
        control_word::disable_voltage : control_word::fault_reset;
    case DriveState::FaultReactionActive:
      return control_word::disable_voltage;
    case DriveState::SwitchOnDisabled:
      return control_word::shutdown;
    case DriveState::ReadyToSwitchOn:
      return previous_control_word == control_word::switch_on ?
        control_word::enable_operation : control_word::switch_on;
    case DriveState::SwitchedOn:
    case DriveState::OperationEnabled:
      return control_word::enable_operation;
    case DriveState::QuickStopActive:
      return control_word::shutdown;
    case DriveState::NotReadyToSwitchOn:
    case DriveState::Unknown:
    default:
      return control_word::disable_voltage;
  }
}

Mode parse_mode(const std::string & mode_name)
{
  std::string m;
  m.reserve(mode_name.size());
  std::transform(mode_name.begin(), mode_name.end(), std::back_inserter(m), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (m == "csp" || m == "8") {
    return Mode::Csp;
  }
  if (m == "csv" || m == "9") {
    return Mode::Csv;
  }
  if (m == "pp" || m == "profile-position" || m == "profile_position" || m == "1") {
    return Mode::ProfilePosition;
  }
  if (m == "pv" || m == "profile-velocity" || m == "profile_velocity" || m == "3") {
    return Mode::ProfileVelocity;
  }
  if (m == "none" || m == "idle" || m == "0") {
    return Mode::NoMode;
  }
  throw std::invalid_argument("unsupported CiA402 mode: " + mode_name);
}

}  // namespace mpk_soem::cia402
