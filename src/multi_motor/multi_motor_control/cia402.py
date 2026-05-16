# -*- coding: utf-8 -*-
"""
cia402.py
---------
CiA 402 state machine / mode constants + Status-Word parser.
Self-contained: no ROS, no Qt imports - safe to unit-test.
"""

from dataclasses import dataclass


# ---------- Control Word (object 0x6040) ----------
class CW:
    DISABLE_VOLTAGE  = 0x0000
    QUICK_STOP       = 0x0002
    SHUTDOWN         = 0x0006
    SWITCH_ON        = 0x0007
    ENABLE_OPERATION = 0x000F
    FAULT_RESET      = 0x0080   # rising edge on bit 7
    HALT_BIT         = 0x0100
    NEW_SET_POINT    = 0x0010   # PP mode: bit 4 rising edge
    CHANGE_IMMEDIATE = 0x0020   # PP mode: bit 5


# ---------- Modes of Operation (object 0x6060/0x6061) ----------
class MODE:
    NO_MODE           = 0
    PROFILE_POSITION  = 1
    PROFILE_VELOCITY  = 3
    PROFILE_TORQUE    = 4
    HOMING            = 6
    INTERPOLATED_POS  = 7
    CSP               = 8
    CSV               = 9
    CST               = 10

    NAMES = {
        0:  "No Mode",
        1:  "Profile Position",
        3:  "Profile Velocity",
        4:  "Profile Torque",
        6:  "Homing",
        7:  "Interpolated Pos",
        8:  "CSP",
        9:  "CSV",
        10: "CST",
    }


# ---------- Status Word (object 0x6041) ----------
_STATE_MASK     = 0x006F  # bits 0,1,2,3,5,6
# Canonical masked values for each CiA 402 state:
_STATE_NAMES = [
    ("Not Ready To Switch On", 0x0000, 0x004F),
    ("Switch On Disabled",     0x0040, 0x004F),
    ("Ready To Switch On",     0x0021, 0x006F),
    ("Switched On",            0x0023, 0x006F),
    ("Operation Enabled",      0x0027, 0x006F),
    ("Quick Stop Active",      0x0007, 0x006F),
    ("Fault Reaction Active",  0x000F, 0x004F),
    ("Fault",                  0x0008, 0x004F),
]


@dataclass
class DriveStatus:
    raw: int
    state: str
    ready_to_switch_on: bool
    switched_on: bool
    operation_enabled: bool
    fault: bool
    voltage_enabled: bool
    quick_stop: bool          # bit 5 = 1 means NOT in quick-stop
    switch_on_disabled: bool
    warning: bool
    target_reached: bool
    internal_limit: bool


def parse_status_word(sw: int) -> DriveStatus:
    """Decode a 16-bit Status Word into named flags + CiA 402 state."""
    sw &= 0xFFFF
    state = "Unknown"
    for name, value, mask in _STATE_NAMES:
        if (sw & mask) == value:
            state = name
            break
    return DriveStatus(
        raw=sw,
        state=state,
        ready_to_switch_on = bool(sw & (1 << 0)),
        switched_on        = bool(sw & (1 << 1)),
        operation_enabled  = bool(sw & (1 << 2)),
        fault              = bool(sw & (1 << 3)),
        voltage_enabled    = bool(sw & (1 << 4)),
        quick_stop         = bool(sw & (1 << 5)),
        switch_on_disabled = bool(sw & (1 << 6)),
        warning            = bool(sw & (1 << 7)),
        target_reached     = bool(sw & (1 << 10)),
        internal_limit     = bool(sw & (1 << 11)),
    )


def next_control_word(current_sw: int) -> int:
    """
    Given the current Status Word, return the Control Word that
    advances the CiA 402 state machine one step toward
    Operation Enabled. Returns CW.ENABLE_OPERATION once the
    drive is already in Operation Enabled.
    """
    s = parse_status_word(current_sw)
    if s.fault:
        return CW.FAULT_RESET
    if s.switch_on_disabled:
        return CW.SHUTDOWN            # -> Ready to Switch On
    if s.ready_to_switch_on and not s.switched_on:
        return CW.SWITCH_ON           # -> Switched On
    if s.switched_on and not s.operation_enabled:
        return CW.ENABLE_OPERATION    # -> Operation Enabled
    return CW.ENABLE_OPERATION
