# MPK_SOEM

`MPK_SOEM` 是一个基于 SOEM 的独立 EtherCAT 四电机控制程序，用来替代之前 `multi_motor` / `MPKEthercat_ws` 中基于 IgH + ROS2 的底层 EtherCAT 通信部分。

当前工程不依赖 ROS2 控制器，直接通过 SOEM 访问网卡、扫描 EtherCAT 从站、配置 PDO/SDO，并周期性下发电机控制命令。

## 当前功能

- 支持 4 个 CiA402 电机驱动器，从站位置默认为 EtherCAT 总线上的 1 到 4。
- 支持 EtherCAT 状态切换：初始化网卡、扫描从站、进入 `SAFE_OP`，再请求进入 `OP`。
- 支持 CiA402 状态机：
  - Fault reset
  - Shutdown
  - Switch on
  - Enable operation
- 支持控制模式：
  - `csp`：Cyclic Synchronous Position，周期同步位置模式
  - `csv`：Cyclic Synchronous Velocity，周期同步速度模式
  - `pp`：Profile Position，轮廓位置模式
  - `pv`：Profile Velocity，轮廓速度模式
  - `idle`：空闲监控模式，不主动使能电机
- 支持默认 1 kHz 控制周期。
- 支持 DC SYNC0，同步周期默认 1 ms。
- 支持 WKC 检查和简单从站恢复逻辑。
- 支持打印每个电机的状态字、CiA402 状态、实际位置、实际速度、模式显示、换算后的物理位置和物理速度。
- 集成 SOEM 示例工具 `slaveinfo`，用于查看 EtherCAT 从站信息、PDO 映射和 SDO 对象字典。

## PDO / SDO

RxPDO，也就是主站写给驱动器的数据：

| 对象 | 名称 | 说明 |
|---|---|---|
| `0x6040` | Control Word | CiA402 控制字 |
| `0x607A` | Target Position | 目标位置 |
| `0x60FF` | Target Velocity | 目标速度 |
| `0x6060` | Mode of Operation | 运行模式 |

TxPDO，也就是驱动器反馈给主站的数据：

| 对象 | 名称 | 说明 |
|---|---|---|
| `0x6041` | Status Word | CiA402 状态字 |
| `0x6064` | Position Actual Value | 实际位置 |
| `0x606C` | Velocity Actual Value | 实际速度 |
| `0x6061` | Mode of Operation Display | 当前运行模式 |

Novanta Denali XCR / Summit 设备上，`0x1600` 和 `0x1A00` 的 PDO 映射通常已经存在。程序启动时不会清空并重写这些 PDO map，而是只按标准 PDO assignment 流程修正 TxPDO 分配：

```text
0x1C13:00 = 0
0x1C13:01 = 0x1A00
0x1C13:00 = 1
```

如果 `slaveinfo -map` 看到 `SM3 inputs` 为空、`Input size: 0bits`，同时 AL 状态码是 `0x0018 No valid inputs available`，通常就是 `0x1C13:01` 没有分配到 `0x1A00`。

启动时写入的 SDO 参考了 `MPKEthercat_ws/src/mpk_ethercat_test/config/motor_config.yaml`：

| 对象 | 默认值 | 说明 |
|---|---:|---|
| `0x60C2:01` | `1` | 插补时间数值 |
| `0x60C2:02` | `-3` | 插补时间基准，表示 `10^-3 s` |
| `0x6081:00` | `500` | Profile Velocity |
| `0x6083:00` | `5000` | Profile Acceleration |
| `0x6084:00` | `5000` | Profile Deceleration |

## SOEM 来源

本目录已经包含 SOEM 源码：

```text
MPK_SOEM/third_party/SOEM
```

当前 vendored SOEM commit 为：

```text
b410bf6
```

注意：当前 SOEM 是 GPLv3 / commercial 双许可证。如果后续要发布或商用，需要确认许可证是否符合项目要求。

## 构建

```bash
cd /home/zjf/MPK_SDK/MPK_SOEM
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

GUI 使用 Qt5 Widgets。如果配置阶段提示找不到 Qt5，可以安装：

```bash
sudo apt install qtbase5-dev
```

如果以后删除了 `third_party/SOEM`，可以重新拉取：

```bash
cd /home/zjf/MPK_SDK/MPK_SOEM
mkdir -p third_party
git clone https://github.com/OpenEtherCATsociety/SOEM.git third_party/SOEM
```

如果 SOEM 安装在其他路径，也可以配置时指定：

```bash
cmake -S . -B build -DSOEM_ROOT=/path/to/soem/install
```

## 查看网卡名称

先确认 EtherCAT 使用的网卡名，例如 `eno1`、`eth0`：

```bash
ip link
```

后面的命令都需要把 `eno1` 替换成你的实际 EtherCAT 网卡。

## 查看从站信息

查看 EtherCAT 从站基本信息：

```bash
sudo ./build/mpk_soem_slaveinfo eno1
```

查看 PDO 映射：

```bash
sudo ./build/mpk_soem_slaveinfo eno1 -map
```

查看 CoE / SDO 对象字典：

```bash
sudo ./build/mpk_soem_slaveinfo eno1 -sdo
```

`-sdo` 输出会比较长，也可能比较慢。

`slaveinfo` 输出中的 EtherCAT 状态数值常见含义：

| 数值 | EtherCAT 状态 |
|---:|---|
| `1` | `INIT` |
| `2` | `PRE_OP` |
| `4` | `SAFE_OP` |
| `8` | `OP` |
| `18` / `0x12` | `PRE_OP + ERROR` |
| `20` / `0x14` | `SAFE_OP + ERROR` |

如果看到类似：

```text
Slave:1
 Name:...
 State: 4
 Man: 0000029c ID: 03831002
```

表示第 1 个从站当前处于 `SAFE_OP`。

## 运行控制程序

启动图形界面：

```bash
sudo -E ./build/mpk_soem_gui
```

GUI 可以选择 `PP`、`PV`、`CSP`、`CSV`、`Idle`，设置 4 个电机目标值、使能掩码、控制周期、持续时间、DC SYNC0，并实时显示每个电机的状态字、CiA402 状态、位置、速度、模式显示、物理位置和物理速度。`Enable selected drives` 只会使能勾选的电机，其余电机保持 EtherCAT OP，但控制字保持 disable voltage。

空闲监控模式，不使能电机：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode idle
```

CSP 模式，使能 4 个电机并保持目标位置为 0：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode csp --target 0,0,0,0 --enable
```

CSV 模式，让第 1 个电机速度目标为 100，运行 5 秒：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode csv --target 100,0,0,0 --enable --duration 5
```

只使能第 1 个电机，其他从站保持 EtherCAT OP 但不执行 CiA402 使能：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode csv --target 100,0,0,0 --enable --enable-mask 0x1 --duration 5
```

CSP 模式，给第 1 个电机目标位置 10000，并限制斜坡速度为 2000 counts/s：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode csp --target 10000,0,0,0 --csp-speed 2000 --enable
```

## 命令参数

| 参数 | 说明 |
|---|---|
| `--ifname <网卡名>` | EtherCAT 网卡名称，例如 `eno1` |
| `--mode <csp/csv/pp/pv/idle>` | 控制模式 |
| `--target <v1,v2,v3,v4>` | 4 个电机的目标值 |
| `--enable` | 自动执行 CiA402 使能流程 |
| `--enable-mask <mask>` | 只使能指定电机，bit0 到 bit3 对应电机 1 到 4，默认 `0x0f` |
| `--duration <秒>` | 运行指定时间后退出 |
| `--rate-hz <频率>` | 控制周期频率，默认 1000 Hz |
| `--csp-speed <counts/s>` | CSP 目标位置斜坡限制 |
| `--print-every <周期数>` | 每隔多少个周期打印一次状态 |
| `--no-dc` | 不启用 DC SYNC0 |
| `--no-id-check` | 跳过 Vendor ID / Product ID 检查 |

## 目标值单位

当前 `--target` 使用的是驱动器对象字典的原始单位，不是物理单位。

位置模式：

- `csp`
- `pp`

`--target` 会写入 `0x607A Target Position`，单位是驱动器位置计数，也就是和 `0x6064 Position Actual Value` 一致的编码器/驱动器 count。

速度模式：

- `csv`
- `pv`

`--target` 会写入 `0x60FF Target Velocity`，单位是驱动器速度原始单位，也就是和 `0x606C Velocity Actual Value` 对应的单位。

当前程序只在打印反馈时做物理量换算：

```cpp
physical_position_mm =
  (position_bias_count - actual_position) * screw_lead_mm_per_rev / encoder_counts_per_rev;

physical_velocity_mm_s =
  -actual_velocity * screw_lead_mm_per_rev / 1000.0;
```

默认换算参数：

```text
encoder_counts_per_rev = 131072
screw_lead_mm_per_rev = 10.0
```

## 默认驱动器型号检查

程序默认检查从站身份：

```text
vendor_id  = 0x0000029c
product_id = 0x03831002
```

如果调试其他驱动器，可以临时跳过检查：

```bash
sudo ./build/mpk_soem_four_motor --ifname eno1 --mode idle --no-id-check
```

## 注意事项

- SOEM 使用 raw socket 访问网卡，通常需要 `sudo` 或 `CAP_NET_RAW` 权限。
- EtherCAT 网卡最好不要被 NetworkManager、DHCP 或普通 IP 网络占用。
- 电机真实运动前，建议先运行 `idle` 模式和 `slaveinfo` 确认从站数量、型号和状态。
- 第一次使能时建议目标值设为当前位置附近，避免 CSP / PP 出现大位置阶跃。
- 如果要做更稳定的实时控制，建议使用实时内核、CPU 隔离和固定优先级线程。
