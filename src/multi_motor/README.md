# multi_motor_control

ROS 2 Humble package that drives a cluster of **Novanta Denali XCR** CiA 402
servo drives over EtherCAT through
[ICube-Robotics/ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2),
plus a **rqt** panel for interactive commissioning.

Supported control modes (switchable at runtime from the GUI):

| Mode | CiA 402 value | Controller (ros2_control)          | Write target object |
|------|--------------|------------------------------------|---------------------|
| PP   | 1            | `forward_command_controller/ForwardCommandController` | 0x607A |
| CSP  | 8            | `position_controllers/JointGroupPositionController` | 0x607A |
| CSV  | 9            | `velocity_controllers/JointGroupVelocityController` | 0x60FF |
| PV   | 3            | `forward_command_controller/ForwardCommandController` | 0x60FF |

Motor count is parametric: `num_joints` defaults to **5**. In the default
layout `joint_1` is a ball-screw linear axis in mm, while `joint_2..joint_5`
are the four rotary axes.

---

## Package layout

```
multi_motor_control/
├── multi_motor_control/              # Python module (rqt plugin code)
│   ├── __init__.py
│   ├── cia402.py                     # state-machine helpers (pure Python)
│   ├── my_motor_rqt_plugin.py        # RosBridge + MultiMotorWidget + Plugin
│   └── unit_converter.py             # ROS degree API <-> drive counts
├── launch/
│   └── multi_motor_control.launch.py
├── config/
│   ├── controllers.yaml              # controller_manager parameters
│   └── ethercat_system.yaml          # Denali XCR slave config (CiA 402)
├── urdf/
│   ├── multi_motor.urdf.xacro        # top-level URDF w/ args
│   └── robot.ros2_control.xacro      # <ros2_control> macro
├── resource/
│   └── multi_motor_control           # ament resource marker
├── plugin.xml                        # rqt plugin manifest
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Prerequisites

1. **IgH EtherCAT master** kernel module loaded, with `ethercat` user-space
   tools installed (`ethercat slaves` should see the Denali drives).
2. `ethercat_driver_ros2` built in the same colcon workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone -b humble https://github.com/ICube-Robotics/ethercat_driver_ros2.git
   ```
3. Standard ros2_control stack from apt:
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                    ros-humble-controller-manager \
                    ros-humble-forward-command-controller \
                    ros-humble-joint-state-broadcaster \
                    ros-humble-control-msgs \
                    ros-humble-rqt-gui ros-humble-rqt-gui-py
   ```

---

## Build

```bash
cd ~/ros2_ws/src
ln -s /home/zjf/MPK_SDK/multi_motor multi_motor_control   # or rename the dir
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_motor_control
source install/setup.bash
```

---

## Run

```bash
# 1) Bring up master, hardware interface and all controllers:
ros2 launch multi_motor_control multi_motor_control.launch.py

# Optional overrides:
#   num_joints:=5
#   master_id:=0
#   urdf_file:=multi_motor.urdf.xacro
#   controllers_file:=controllers.yaml
#   slave_config_file:=ethercat_system.yaml
#   encoder_resolution:=865075.2
#   position_limits_file:=/path/to/calibrate.json

# 2) In another terminal, open the GUI
rqt --force-discover
#    -> menu: Plugins -> Robot -> "Multi Motor Control"

#    or launch the widget standalone (no rqt):
ros2 run multi_motor_control my_motor_rqt_plugin
```

On start-up **all drives stay in "Switch On Disabled"** because the slave
YAML sets Control Word `default: 0` and `cia402_cmd_controller` claims the
`control_word` command interface with no user command yet. The GUI has to
explicitly click **Enable** to walk the CiA 402 state machine.

---

## Controller topology

```
/joint_states                            (from joint_state_broadcaster)
/dynamic_joint_states                    (adds status_word, mode display)

/cia402_cmd_controller/commands          -> 0x6040 Control Word (float64 -> uint16)
/cia402_mode_controller/commands         -> 0x6060 Modes of Operation (float64 -> int8)
/fault_reset_controller/commands         -> reset_fault interface (auxiliary pulse)

/pp_controller/commands                  -> 0x607A Target Position in PP mode (when active)
/csp_controller/commands                 -> 0x607A Target Position  (when active)
/csv_controller/commands                 -> 0x60FF Target Velocity  (when active)
/pv_controller/commands                  -> 0x60FF Target Velocity in PV mode (when active)

/controller_manager/list_controllers     (srv)
/controller_manager/switch_controller    (srv)
```

`pp_controller`, `csp_controller`, `csv_controller`, `pv_controller` are **mutually exclusive**
and spawned **inactive**. The GUI calls `/controller_manager/switch_controller`
under the hood when you click *Apply* after picking a mode. GUI motion
setpoints are entered in degrees and published through the ROS degree API
below; `unit_converter` forwards them to the low-level controller topics.

---

## ROS position API

The EtherCAT/CiA 402 layer still uses drive-native `counts` and `counts/s`.
For higher-level software, `multi_motor_control.launch.py` also starts
`unit_converter`, which assumes a 17-bit motor encoder with a 6.6:1 gearbox
by default. The conversion uses the effective output-shaft resolution:

```text
encoder_resolution = 131072 * 6.6 = 865075.2 counts/output-rev
```

Command topics keep their historical names. Values are interpreted per joint:
`joint_1` uses mm/mm/s when its `calibrate.json` entry has `unit: "mm"`;
rotary joints use deg/deg/s.

```text
/multi_motor/pp_position_deg/commands       -> /pp_controller/commands
/multi_motor/csp_position_deg/commands      -> /csp_controller/commands
/multi_motor/csv_velocity_deg_s/commands    -> /csv_controller/commands
/multi_motor/pv_velocity_deg_s/commands     -> /pv_controller/commands
```

All command topics use `std_msgs/msg/Float64MultiArray` with one value per
joint, ordered as `joint_1 ... joint_N`. Feedback is published as
`control_msgs/msg/DynamicJointState`:

```text
/multi_motor/states_deg
  interface_names:
    [position_deg|position_mm, velocity_deg_s|velocity_mm_s, raw_position_cnt,
     restored_position_cnt, restored_position_deg|restored_position_mm,
     restore_offset_cnt, turn, single_cnt]
```

Rotary conversion is:

```text
counts   = round(deg   * encoder_resolution / 360.0)
deg      = counts      * 360.0 / encoder_resolution
counts/s = round(deg/s * encoder_resolution / 360.0)
deg/s    = counts/s    * 360.0 / encoder_resolution
```

The default linear conversion for `joint_1` comes from the static joint
geometry configuration:

```text
encoder_counts_per_rev = 131072
screw_lead_mm_per_rev = 10.0
mm = (count - zero_position_cnt) * linear_direction * 10.0 / 131072
```

The degree API only converts units. Drive enabling, mode selection, and
controller activation are still handled through the CiA 402 command/mode
controllers and `/controller_manager/switch_controller`. The rqt/standalone
GUI also uses these degree command and feedback topics.

### Restored encoder counts

The Denali XCR `0x6064` position is published by `joint_state_broadcaster`.
The launch file remaps that raw stream to:

```text
/joint_states_raw
```

`unit_converter` subscribes to `/joint_states_raw`, restores the continuous
encoder count using the saved turn offset, and republishes:

```text
/joint_states
  position = raw 0x6064 count
  velocity = raw 0x606C count/s
  effort   = restored continuous count
```

The persisted state file defaults to:

```text
<workspace>/state/multi_motor_encoder_state.json
```

With the default Denali 17-bit encoder configuration:

```text
encoder_one_turn_cnt = 131072
```

On startup, the restored count is aligned to the saved continuous count by
adding an integer multiple of `encoder_one_turn_cnt`. During operation the
state file is written atomically every `0.02 s` when the restored count has
changed.

To avoid overwriting good state during a drive/encoder power cycle, encoder
state updates are gated by each joint's CiA 402 `status_word`: when the status
word is zero or unavailable, the last restored position is held in memory and
the state file is not written. The status word must also be fresh within
`encoder_status_timeout_sec` so stale pre-power-loss status cannot approve new
raw samples. When feedback becomes valid again, `unit_converter` waits at
least `encoder_powerup_stabilize_sec` and then requires raw `0x6064` to stay
within `encoder_raw_stable_delta_counts` for
`encoder_raw_stable_duration_sec` before updating restored state. A raw
`0x6064` jump larger than half an encoder turn then triggers re-alignment
against the last trusted continuous count; file writes are delayed by
`encoder_realign_save_holdoff_sec` after that re-alignment to avoid saving a
transient reset value. This protects normal and most sudden stop cases, but no
software-only method can recover motion that happens while the encoder and
computer are both unpowered.

### Position limits

Joint position limits are stored in `calibrate.json`:

```json
{
  "joint_1": {
    "axis_type": "linear",
    "unit": "mm",
    "min_mm": 0.0,
    "max_mm": 100.0,
    "range_mm": 100.0,
    "default_mm": 0.0,
    "zero_position_cnt": 0.0,
    "encoder_counts_per_rev": 131072.0,
    "screw_lead_mm_per_rev": 10.0,
    "linear_direction": -1.0
  },
  "joint_2": {
    "min_deg": 0.0,
    "max_deg": 147.4,
    "range_deg": 147.4,
    "default_deg": 0.0,
    "home_offset_deg": 0.0
  }
}
```

The GUI uses each joint's calibrated min/max for the PP/CSP position slider
and spinbox range. `unit_converter` also loads the same file and clamps
position commands received on:

```text
/multi_motor/pp_position_deg/commands
/multi_motor/csp_position_deg/commands
```

Those PP/CSP position command topics are interpreted in the restored
calibrated position coordinate system. In other words, a command value should
match the `restored_position_deg` or `restored_position_mm` feedback value you
want the joint to reach. Before
publishing to `/pp_controller/commands` or `/csp_controller/commands`,
`unit_converter` converts it with:

```text
restored_raw_deg = command_deg - home_offset_deg
restored_target_cnt = deg_to_counts(restored_raw_deg)
drive_target_cnt = restored_target_cnt - restore_offset_cnt
```

If the restored encoder state for a joint is not ready, the PP/CSP command
frame is rejected instead of falling back to a raw count that could jump.

That means code-based control should publish calibrated position commands to the
`/multi_motor/*_position_deg/commands` topics to get the same limit
protection as the GUI. If code publishes directly to the lower-level
`/pp_controller/commands` or `/csp_controller/commands` count topics, it must
apply the limits and restored-count offset before converting to counts.

When the GUI aligns PP/CSP sliders to the current pose, switches into CSP, or
fills non-streaming joints in a CSP command vector, it uses
`restored_position_deg`/`restored_position_mm` first and falls back to
`position_deg`/`position_mm` only before restored feedback is available. Raw
limit calibration still uses the raw position path directly.

The GUI has **Enable All** and **Disable All** toolbar buttons for batch state
control. **Enable All** staggers the normal per-joint enable sequence so the
drives do not all transition at the exact same instant. **Disable All** stops
calibration/CSP streaming and sends Disable Voltage to every joint.

The GUI has a **Calibrate Linear M1** button for the first ball-screw axis.
It does not send motion commands. Click it, manually move `joint_1` through
its travel, then click **Save Linear M1**. The minimum physical position is
stored as `0 mm`, and the measured travel is written as `range_mm/max_mm`.

The GUI has a **Calibrate Rotary Zero M2-M5** button for initial zeroing of
the four rotary axes. It requires those four joints to be enabled, switches
them to CSP position mode, and ramps the raw
position target in the negative direction at `-8 deg/s` by default. During
calibration the GUI publishes directly to `/csp_controller/commands` in
drive-native counts so the unhomed degree limits do not clamp the motion. A
case where the target position is clearly ahead of feedback while feedback
position stops progressing is treated as a minimum-limit hit, even if the
joint started very close to that limit and barely moved. A near-zero velocity
fault with a small negative target lead is also treated as a minimum-limit
hit. Each joint is saved as soon as it reaches the minimum limit, then its CSP
target is held while the remaining joints continue.
For each completed joint, the GUI writes `home_offset_deg` and rewrites the
usable range as `min_deg=0`, `max_deg=range_deg`. `default_deg` is treated as
an explicit restored-coordinate absolute pose and is not derived from zero
calibration.
Zero calibration saves `home_offset_deg = -raw_min`, so the detected minimum
mechanical limit maps directly to calibrated `0 deg`.
After saving each joint, the GUI publishes that raw zero count to
`/multi_motor/reset_encoder_restore`. `unit_converter` then clears that
joint's restored encoder turn offset, sets its restored continuous count equal
to the current raw count, and immediately rewrites
`state/multi_motor_encoder_state.json`. This keeps the GUI's
`Restored Position` at the same zero as the newly calibrated raw position.

The GUI also has a **Calibrate Rotary Range M2-M5** button for measuring the
usable rotary travel.
It uses the same raw CSP path, first ramping each enabled joint in the negative
direction until a limit-like stall is detected, then reversing that joint in
the positive direction until the second limit-like stall. It updates the
stored travel length and the software maximum around the existing zero:

```text
range_deg = raw_max - raw_min
min_deg = 0
max_deg = range_deg
```

Each joint switches from the negative search to the positive search as soon as
its negative limit is detected. Completed joints have their range saved and are
held while the remaining joints continue. If a drive faults at the positive end
after valid motion, the fault can be treated as the positive limit and saved;
faults during the negative search abort the range calibration because the drive
must be manually recovered before reversing safely. Range calibration does not
rewrite `home_offset_deg`; run **Calibrate Rotary Zero M2-M5** again only when the mechanical
zero/minimum-limit reference itself has changed.

The **Go Default** toolbar button switches M2-M5 to CSP position control and
sends only those four rotary joints to their saved `default_deg`. M1 is held
at its current feedback position.

This stall-based limit detection is an inference, not a sensor. A physical
limit switch, drive torque/current threshold, or CiA 402 homing method is more
reliable and safer for production use.

---

## CiA 402 state-machine walk (what the GUI does on "Enable")

The widget polls the current Status Word at 20 Hz and writes the next
Control Word step until **Operation Enabled** is reached:

```
current Status Word  ->   next Control Word
-------------------       -----------------
Fault                ->   0x0080 (Fault Reset, bit 7 rising edge)
Switch On Disabled   ->   0x0006 (Shutdown)
Ready To Switch On   ->   0x0007 (Switch On)
Switched On          ->   0x000F (Enable Operation)
Operation Enabled    ->   0x000F (hold)
```

The same logic lives in `multi_motor_control/cia402.py:next_control_word`
and is 100 % unit-testable without ROS.

---

## Quick troubleshooting

| Symptom                                           | Likely cause                                         |
|---------------------------------------------------|------------------------------------------------------|
| `controller_manager` exits with *"No hardware_interface plugin found"* | `ethercat_driver_ros2` not in the same workspace or not sourced |
| Drives auto-enable before the GUI opens           | `default: 0` missing from Control Word channel in `ethercat_system.yaml` |
| GUI shows `status_word: 0x0000` forever           | `joint_state_broadcaster` not publishing `/dynamic_joint_states` - check that custom state interfaces are declared on each `<joint>` |
| "Apply CSV" succeeds but motor does not move      | Drive still in Switch-On-Disabled - click Enable first; or mode 9 not supported by firmware build |
| `switch_controller service not ready` in log      | `--controller-manager-timeout` too small, bump in launch file |
| Multiple motors conflict on command interface     | Two motion controllers active at once - only one should be active; click *Apply* to let the GUI call `switch_controller` with `deactivate` set |

---

## Extending Beyond 5 Motors

1. `ros2 launch multi_motor_control multi_motor_control.launch.py num_joints:=8`
2. Append `joint_6 ... joint_8` to every `joints:` list in
   `config/controllers.yaml` (all controller `joints:` lists).
3. Adjust `DEFAULT_JOINTS` in `multi_motor_control/my_motor_rqt_plugin.py`
   or pass a custom `joint_names` list when constructing
   `MultiMotorWidget`.

No changes are needed to the slave YAML, xacro, or plugin code itself -
xacro recursion generates `joint_1..joint_N` from the single `num_joints`
argument.
