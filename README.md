# MPKEthercat_ws

EtherCAT motor-control codebase for MPK/Denali XCR CiA 402 drives.

Maintained by **zjf**.

The maintained implementation uses the IgH EtherCAT master, `ros2_control`,
and an rqt GUI under ROS 2.

## Repository Layout

```text
.
├── src/
│   ├── ethercat_driver_ros2/    # EtherCAT ROS 2 driver stack, vendored as source
│   └── multi_motor/             # Main ROS 2 multi-motor control package
├── docs/                        # Project notes and integration docs
├── legacy/                      # Archived early test code, not tracked/uploaded
└── build/ install/ log/         # Local colcon outputs, ignored by Git
```

## ROS 2 + IgH + ros2_control

Main package:

```text
src/multi_motor
```

This package is installed as `multi_motor_control`. It controls five Denali XCR
drives by default: one ball-screw linear axis and four rotary axes. It supports:

| Mode | CiA 402 value | Target object |
|------|---------------|---------------|
| PP   | 1             | `0x607A` Target Position |
| PV   | 3             | `0x60FF` Target Velocity |
| CSP  | 8             | `0x607A` Target Position |
| CSV  | 9             | `0x60FF` Target Velocity |

The GUI uses `/dynamic_joint_states` for drive status and the ROS-level API
under `/multi_motor/*` for motion commands and feedback. The launch file starts
`unit_converter`, which converts the linear axis to mm and the rotary axes to
degrees before forwarding commands to the low-level ros2_control topics.

### ROS 2 Build

From the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
```

To build only the main application and driver stack:

```bash
colcon build --symlink-install --packages-select \
  ethercat_interface ethercat_driver ethercat_generic_slave \
  ethercat_generic_cia402_drive ethercat_msgs ethercat_manager \
  ethercat_driver_ros2 multi_motor_control
source install/setup.bash
```

### ROS 2 Run

Bring up the hardware and controllers:

```bash
ros2 launch multi_motor_control multi_motor_control.launch.py
```

Open the GUI in another terminal:

```bash
source install/setup.bash
ros2 run multi_motor_control my_motor_rqt_plugin
```

or through rqt:

```bash
rqt --force-discover
```

then select `Plugins -> Robot -> Multi Motor Control`.

More details are in:

```text
src/multi_motor/README.md
```

## Important Directories

`src/ethercat_driver_ros2`

: Vendored ROS 2 EtherCAT driver stack. This repository keeps it as normal
  source code rather than a Git submodule, because the local project contains
  drive-specific changes such as Profile Position command handling.

`src/multi_motor`

: Main maintained ROS 2 application. It contains the launch files, xacro URDF,
  EtherCAT slave YAML, controller YAML, and rqt/standalone GUI.

`legacy`

: Old early-stage test package. It is kept locally for reference, but ignored
  for Git upload and colcon builds.

## Git / Upload Policy

Tracked source should include:

- `README.md` and source/config files
- `src/multi_motor/`
- `src/ethercat_driver_ros2/`
- project config files such as `.gitignore`

Ignored local/generated data:

- `build/`
- `install/`
- `log/`
- `docs/`
- `legacy/`
- `src/MPK_SOEM/`
- editor and assistant state: `.vscode/`, `.codex/`, `.agents/`
- Python caches and compiled artifacts

The root `.gitignore` is configured for this layout.

## Troubleshooting Notes

- If the GUI shows `status_word = 0x0000` for every motor, first check whether
  EtherCAT slaves reached OP state. This usually points below the GUI layer.
- `/dynamic_joint_states` must contain `status_word` and
  `modes_of_operation_display` for the ROS 2 GUI to show drive status.
- `assign_activate: 0x0300` enables DC SYNC0 and is useful for CSP/CSV. For
  PP/PV-only diagnosis, temporarily testing `assign_activate: 0x0000` can help
  separate DC timing issues from wiring/power issues.
- Do not upload `build/`, `install/`, `log/`, or `docs/`; regenerate runtime
  outputs locally and keep private notes outside Git.
