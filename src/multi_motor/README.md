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

Motor count is parametric: `num_joints` defaults to **4** and scales up to **8**
without changing any code, only the `controllers.yaml` joint list.

---

## Package layout

```
multi_motor_control/
├── multi_motor_control/              # Python module (rqt plugin code)
│   ├── __init__.py
│   ├── cia402.py                     # state-machine helpers (pure Python)
│   └── my_motor_rqt_plugin.py        # RosBridge + MultiMotorWidget + Plugin
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
#   num_joints:=6
#   master_id:=0
#   urdf_file:=multi_motor.urdf.xacro
#   controllers_file:=controllers.yaml
#   slave_config_file:=ethercat_system.yaml

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
under the hood when you click *Apply* after picking a mode.

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

## Extending to 8 motors

1. `ros2 launch multi_motor_control multi_motor_control.launch.py num_joints:=8`
2. Append `joint_5 ... joint_8` to every `joints:` list in
   `config/controllers.yaml` (all controller `joints:` lists).
3. Adjust `DEFAULT_JOINTS` in `multi_motor_control/my_motor_rqt_plugin.py`
   or pass a custom `joint_names` list when constructing
   `MultiMotorWidget`.

No changes are needed to the slave YAML, xacro, or plugin code itself -
xacro recursion generates `joint_1..joint_N` from the single `num_joints`
argument.
