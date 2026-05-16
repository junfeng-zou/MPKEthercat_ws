# MPK_SOEM

Standalone SOEM-based EtherCAT controller for four CiA 402 drives. It mirrors the core behavior of the previous IgH/ROS2 setup:

- 4 motor slaves, positions 1..4 on the EtherCAT bus.
- RxPDO: `0x6040` Control Word, `0x607A` Target Position, `0x60FF` Target Velocity, `0x6060` Mode of Operation.
- TxPDO: `0x6041` Status Word, `0x6064` Actual Position, `0x606C` Actual Velocity, `0x6061` Mode Display.
- For Novanta Denali XCR / Summit drives, the controller keeps the existing `0x1600` / `0x1A00` PDO maps and only fixes the TxPDO assignment with `0x1C13:00 = 0`, `0x1C13:01 = 0x1A00`, `0x1C13:00 = 1`.
- Startup SDOs copied from `MPKEthercat_ws/src/mpk_ethercat_test/config/motor_config.yaml`.
- CiA 402 enable sequence and fault-reset handling.
- CSP, CSV, Profile Position and Profile Velocity command modes.

The SOEM initialization and cyclic process-data loop follows the public SOEM sample pattern from OpenEtherCATsociety/SOEM: initialize the NIC, discover slaves, configure PDO mapping, exchange process data, then request OP state.

This directory currently vendors SOEM under `third_party/SOEM` at commit `b410bf6`. Current SOEM is dual-licensed GPLv3/commercial, so keep that in mind if this code will be redistributed.

## Install SOEM

SOEM is already present in `third_party/SOEM`. If you remove it later, either install SOEM system-wide or put the source tree back here:

```bash
cd ~/MPK_SDK/MPKEthercat_ws/src/MPK_SOEM
mkdir -p third_party
git clone https://github.com/OpenEtherCATsociety/SOEM.git third_party/SOEM
```

If SOEM is installed elsewhere, pass `-DSOEM_ROOT=/path/to/soem/install` when configuring CMake. The bundled path is built directly from SOEM source files, so it works with this machine's CMake 3.22 even though upstream SOEM's top-level CMake currently asks for a newer CMake.

## Build

```bash
cd ~/MPK_SDK/MPKEthercat_ws/src/MPK_SOEM
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

The GUI target uses Qt5 Widgets. On Ubuntu, install it with:

```bash
sudo apt install qtbase5-dev
```

## Run

List NIC names first:

```bash
ip link
```

Start the native motor-control GUI:

```bash
sudo -E ./build/mpk_soem_gui
```

Run in monitor/idle mode without enabling the drives:

```bash
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode idle
```

Enable all four drives in CSP and hold zero target counts:

```bash
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode csp --target 0,0,0,0 --enable
```

Run CSV for five seconds:

```bash
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode csv --target 100,0,0,0 --enable --duration 5
```

Enable only motor 1 while keeping all EtherCAT slaves in OP:

```bash
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode csv --target 100,0,0,0 --enable --enable-mask 0x1 --duration 5
```

Use a CSP ramp limit to avoid a step command:

```bash
sudo ./build/mpk_soem_four_motor --ifname enp3s0 --mode csp --target 10000,0,0,0 --csp-speed 2000 --enable
```

## Notes

- The default identity check expects `vendor_id: 0x0000029c` and `product_id: 0x03831002`. Use `--no-id-check` only while commissioning different drives.
- The default cycle is 1 kHz and writes `0x60C2 = 1 * 10^-3 s`.
- Physical position/velocity conversion uses the same defaults as the ROS2 broadcaster: `131072` counts per revolution and `10.0 mm/rev` screw lead.
- SOEM raw-socket access usually requires `sudo` or `CAP_NET_RAW`.
- For hard real-time behavior, run on a tuned Linux kernel and isolate the EtherCAT NIC from NetworkManager.
