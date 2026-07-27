# MPKEthercat_ws

ROS 2 EtherCAT workspace for controlling five Novanta Denali XCR servo drives with the IgH EtherCAT Master and `ros2_control`.

The default machine layout is:

- `joint_1`: ball-screw linear axis, controlled in mm and mm/s.
- `joint_2` to `joint_5`: rotary axes, controlled in degrees and degrees/s.

The project provides a custom rqt control panel, CiA 402 state control, runtime mode switching, joint calibration, software position limits, and continuous encoder-position recovery across drive restarts.

For integration with a higher-level system controller, see the [Chinese control API reference](CONTROL_API_CN.md).

> **Safety:** This software controls real motors. Test with low speed and torque limits, keep an emergency stop available, and verify the mechanical workspace before enabling any drive. Software calibration and limits do not replace hardware safety devices.

## Features

- Five-axis Denali XCR control over EtherCAT.
- PP, CSP, CSV, and PV CiA 402 operating modes.
- Explicit drive enable, disable, and fault reset.
- Mutually exclusive `ros2_control` motion controllers.
- rqt and standalone Qt control interfaces.
- Batch enable/disable and move-to-default commands.
- Linear-axis and rotary-axis calibration tools.
- Per-joint unit conversion and software position limits.
- Persistent multi-turn encoder-position recovery.
- Vendored `ethercat_driver_ros2` with project-specific changes.

## Repository Layout

```text
MPKEthercat_ws/
├── README.md
├── .gitignore
└── src/
    ├── ethercat_driver_ros2/       # Customized ROS 2 EtherCAT driver
    └── multi_motor/                # Main five-axis control package
        ├── config/                 # EtherCAT, controller, and joint settings
        ├── launch/                 # Main and PV diagnostic launch files
        ├── multi_motor_control/    # GUI, CiA 402, limits, and conversion code
        ├── urdf/                   # ros2_control xacro files
        ├── calibrate.json          # Machine-specific calibration values
        ├── package.xml
        └── setup.py
```

Generated directories such as `build/`, `install/`, `log/`, and `state/` are not tracked by Git.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- IgH EtherCAT Master / EtherLab 1.5.x
- `ros2_control` and `ros2_controllers`
- Novanta Denali XCR EtherCAT drives
- Python 3, PyYAML, and rqt

The IgH master must be installed, configured for the EtherCAT network adapter, and running before the ROS 2 system is launched. See [the bundled driver installation guide](src/ethercat_driver_ros2/INSTALL.md) for the IgH and driver setup.

Verify the EtherCAT bus before continuing:

```bash
sudo /etc/init.d/ethercat start
ethercat slaves
```

The default configuration expects five drives in bus order, with `joint_1` mapped to slave position 0 and `joint_5` mapped to slave position 4.

## Install ROS Dependencies

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-control-msgs \
  ros-humble-rqt-gui \
  ros-humble-rqt-gui-py \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

From the workspace root, install any remaining package dependencies:

```bash
# Run these two commands once if rosdep is not initialized yet:
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/MPK_SDK/MPKEthercat_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

To rebuild only the application package:

```bash
colcon build --symlink-install --packages-select multi_motor_control
source install/setup.bash
```

## Run

Start the EtherCAT hardware interface, controllers, state converter, and robot state publisher:

```bash
source /opt/ros/humble/setup.bash
source ~/MPK_SDK/MPKEthercat_ws/install/setup.bash
ros2 launch multi_motor_control multi_motor_control.launch.py
```

All motion controllers are loaded inactive, and the drives are not enabled automatically.

In another terminal, start the standalone control panel:

```bash
source /opt/ros/humble/setup.bash
source ~/MPK_SDK/MPKEthercat_ws/install/setup.bash
ros2 run multi_motor_control my_motor_rqt_plugin
```

The same panel can be opened through rqt:

```bash
rqt --force-discover
```

Select `Plugins -> Robot -> Multi Motor Control`.

For a reduced Profile Velocity diagnostic setup:

```bash
ros2 launch multi_motor_control multi_motor_pv_test.launch.py
```

## Control Modes

| Mode | CiA 402 value | Controller | Target object |
|---|---:|---|---|
| PP | 1 | `pp_controller` | `0x607A` Target Position |
| CSP | 8 | `csp_controller` | `0x607A` Target Position |
| CSV | 9 | `csv_controller` | `0x60FF` Target Velocity |
| PV | 3 | `pv_controller` | `0x60FF` Target Velocity |

Higher-level applications should publish physical-unit commands through the `/multi_motor/*` topics. These commands pass through unit conversion, encoder restoration, and position-limit checks before reaching the low-level controllers.

## Main Configuration Files

- `src/multi_motor/config/ethercat_system.yaml`: Denali XCR PDO, SDO, and Sync Manager configuration.
- `src/multi_motor/config/controllers.yaml`: controller definitions and joint lists.
- `src/multi_motor/config/joint_geometry.yaml`: axis types and encoder/mechanical conversion parameters.
- `src/multi_motor/calibrate.json`: machine-specific limits, zero offsets, and default positions.
- `src/multi_motor/launch/multi_motor_control.launch.py`: system startup and encoder-recovery parameters.

Back up `calibrate.json` before recalibrating the machine. If the number of joints changes, update the controller joint lists, geometry, and calibration files together.

## License

`multi_motor_control` declares the Apache-2.0 license in its package manifest. The bundled EtherCAT driver retains its upstream license in [src/ethercat_driver_ros2/LICENSE](src/ethercat_driver_ros2/LICENSE).
