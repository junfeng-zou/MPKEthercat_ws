# MPKEthercat_ws

EtherCAT motor-control codebase for MPK/Denali XCR CiA 402 drives.

Maintained by **zjf**.

This repository keeps two implementation routes side by side:

- **ROS 2 route**: IgH EtherCAT master + `ros2_control` + rqt GUI.
- **SOEM route**: standalone CMake/SOEM controller without ROS 2.

The ROS 2 route is the maintained daily development path. The SOEM route is
kept as an independent lower-level implementation for comparison, diagnosis,
and non-ROS experiments.

## Repository Layout

```text
.
├── src/
│   ├── ethercat_driver_ros2/    # EtherCAT ROS 2 driver stack, vendored as source
│   ├── multi_motor/             # Main ROS 2 multi-motor control package
│   ├── MPK_SOEM/                # Standalone SOEM route, built with CMake
│   └── ethercat.json            # PDO mapping reference
├── docs/                        # Project notes and integration docs
├── legacy/                      # Archived early test code, not tracked/uploaded
├── build/ install/ log/         # Local colcon outputs, ignored by Git
└── log.md                       # Human-written project/debug notes
```

## Route 1: ROS 2 + IgH + ros2_control

Main package:

```text
src/multi_motor
```

This package is installed as `multi_motor_control`. It controls four Denali XCR
drives by default and supports:

| Mode | CiA 402 value | Target object |
|------|---------------|---------------|
| PP   | 1             | `0x607A` Target Position |
| PV   | 3             | `0x60FF` Target Velocity |
| CSP  | 8             | `0x607A` Target Position |
| CSV  | 9             | `0x60FF` Target Velocity |

The GUI uses `/dynamic_joint_states` for drive status and standard ROS 2
controller topics for commands.

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

## Route 2: Standalone SOEM

Main directory:

```text
src/MPK_SOEM
```

This is not a ROS package and is intentionally skipped by colcon via
`src/MPK_SOEM/COLCON_IGNORE`.

It vendors SOEM under:

```text
src/MPK_SOEM/third_party/SOEM
```

### SOEM Build

```bash
cd src/MPK_SOEM
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

### SOEM Run

SOEM uses raw sockets, so runtime usually needs `sudo` or equivalent
`CAP_NET_RAW` permissions.

```bash
ip link
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode idle
```

Native GUI:

```bash
sudo -E ./build/mpk_soem_gui
```

More details are in:

```text
src/MPK_SOEM/README.md
src/MPK_SOEM/README_CN.md
```

## Important Directories

`src/ethercat_driver_ros2`

: Vendored ROS 2 EtherCAT driver stack. This repository keeps it as normal
  source code rather than a Git submodule, because the local project contains
  drive-specific changes such as Profile Position command handling.

`src/multi_motor`

: Main maintained ROS 2 application. It contains the launch files, xacro URDF,
  EtherCAT slave YAML, controller YAML, and rqt/standalone GUI.

`src/MPK_SOEM`

: Independent SOEM implementation. It is useful for separating EtherCAT/drive
  behavior from ROS 2 and controller-manager behavior.

`legacy`

: Old early-stage test package. It is kept locally for reference, but ignored
  for Git upload and colcon builds.

## Git / Upload Policy

Tracked source should include:

- `README.md`, `log.md`, and `docs/`
- `src/multi_motor/`
- `src/ethercat_driver_ros2/`
- `src/MPK_SOEM/`
- project config files such as `.gitignore`

Ignored local/generated data:

- `build/`
- `install/`
- `log/`
- `legacy/`
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
- Do not upload `build/`, `install/`, or `log/`; regenerate them locally.

## License Notes

The ROS 2 package metadata uses Apache-2.0 where applicable. The SOEM route
vendors SOEM under `src/MPK_SOEM/third_party/SOEM`; check SOEM's upstream
license terms before redistribution or commercial use.
