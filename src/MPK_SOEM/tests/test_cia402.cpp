#include "mpk_soem/cia402.hpp"

#include <cassert>
#include <iostream>

int main()
{
  using namespace mpk_soem::cia402;

  assert(decode_status_word(0x0000) == DriveState::NotReadyToSwitchOn);
  assert(decode_status_word(0x0040) == DriveState::SwitchOnDisabled);
  assert(decode_status_word(0x0021) == DriveState::ReadyToSwitchOn);
  assert(decode_status_word(0x0023) == DriveState::SwitchedOn);
  assert(decode_status_word(0x0027) == DriveState::OperationEnabled);
  assert(decode_status_word(0x0008) == DriveState::Fault);

  assert(next_enable_control_word(0x0040, 0) == control_word::shutdown);
  assert(next_enable_control_word(0x0021, 0) == control_word::switch_on);
  assert(next_enable_control_word(0x0021, control_word::switch_on) == control_word::enable_operation);
  assert(next_enable_control_word(0x0023, 0) == control_word::enable_operation);
  assert(next_enable_control_word(0x0027, 0) == control_word::enable_operation);
  assert(next_enable_control_word(0x0008, 0) == control_word::fault_reset);
  assert(next_enable_control_word(0x0008, control_word::fault_reset) == control_word::disable_voltage);

  assert(parse_mode("csp") == Mode::Csp);
  assert(parse_mode("CSV") == Mode::Csv);
  assert(parse_mode("pp") == Mode::ProfilePosition);
  assert(parse_mode("pv") == Mode::ProfileVelocity);

  std::cout << "test_cia402 passed" << std::endl;
  return 0;
}
