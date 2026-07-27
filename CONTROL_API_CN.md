# 末端执行器底层控制接口说明

本文说明 `MPKEthercat_ws` 当前已经实现的底层控制能力，包括 ROS 2 话题、服务、Python 函数、控制器选项、单位换算、标定和编码器恢复逻辑。

本文只描述项目中已经存在的实现。

当前实现对应以下文件：

- `src/multi_motor/multi_motor_control/cia402.py`
- `src/multi_motor/multi_motor_control/joint_limits.py`
- `src/multi_motor/multi_motor_control/unit_converter.py`
- `src/multi_motor/multi_motor_control/my_motor_rqt_plugin.py`
- `src/multi_motor/config/controllers.yaml`
- `src/multi_motor/config/ethercat_system.yaml`
- `src/multi_motor/config/joint_geometry.yaml`
- `src/multi_motor/calibrate.json`
- `src/multi_motor/launch/multi_motor_control.launch.py`

## 1. 系统组成

项目使用：

```text
上层 ROS 节点 / rqt GUI
        ↓
物理单位命令话题
        ↓
unit_converter
        ↓
ros2_control 控制器
        ↓
定制 ethercat_driver_ros2
        ↓
IgH EtherCAT Master
        ↓
5 × Novanta Denali XCR
```

启动命令：

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 launch multi_motor_control multi_motor_control.launch.py
```

主 launch 启动：

- `ros2_control_node`
- `robot_state_publisher`
- `multi_motor_unit_converter`
- `joint_state_broadcaster`
- `cia402_cmd_controller`
- `cia402_mode_controller`
- `fault_reset_controller`
- `pp_controller`
- `csp_controller`
- `csv_controller`
- `pv_controller`

其中：

- 状态、控制字、模式和故障复位控制器启动为 `active`；
- PP、CSP、CSV、PV 四个运动控制器只加载，初始状态为 `inactive`；
- Control Word 默认值为 0，电机不会随系统启动自动使能。

## 2. 关节顺序与单位

当前默认关节顺序固定为：

```text
[joint_1, joint_2, joint_3, joint_4, joint_5]
```

所有 `Float64MultiArray` 控制命令都必须包含完整的 5 个元素，并使用上述顺序。

| 关节 | 机构 | 位置单位 | 速度单位 | 当前标定范围 | 当前默认位置 |
|---|---|---|---|---:|---:|
| `joint_1` | 滚珠丝杠直线轴 | mm | mm/s | 0 ～ 264.8367 mm | 0 mm |
| `joint_2` | 旋转轴 | deg | deg/s | 0 ～ 353.2292 deg | 266.3473 deg |
| `joint_3` | 旋转轴 | deg | deg/s | 0 ～ 353.2400 deg | 60.6616 deg |
| `joint_4` | 旋转轴 | deg | deg/s | 0 ～ 352.6212 deg | 139.9350 deg |
| `joint_5` | 旋转轴 | deg | deg/s | 0 ～ 353.5658 deg | 23.9044 deg |

这些数值来自当前 `src/multi_motor/calibrate.json`。重新标定会直接修改该文件。

部分话题名称包含 `_deg`，这是历史命名。对 `joint_1`，其中的数值仍然是 mm 或 mm/s。

## 3. 当前 ROS 2 接口总览

### 3.1 状态话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | CiA 402 状态字、当前模式和底层接口 |
| `/joint_states_raw` | `sensor_msgs/msg/JointState` | 驱动器原始位置/速度计数 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 原始计数和恢复后的连续计数 |
| `/multi_motor/states_deg` | `control_msgs/msg/DynamicJointState` | 物理单位、原始计数和恢复位置 |

### 3.2 物理单位命令话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/multi_motor/pp_position_deg/commands` | `std_msgs/msg/Float64MultiArray` | PP 位置 |
| `/multi_motor/csp_position_deg/commands` | `std_msgs/msg/Float64MultiArray` | CSP 位置 |
| `/multi_motor/csv_velocity_deg_s/commands` | `std_msgs/msg/Float64MultiArray` | CSV 速度 |
| `/multi_motor/pv_velocity_deg_s/commands` | `std_msgs/msg/Float64MultiArray` | PV 速度 |
| `/multi_motor/reset_encoder_restore` | `std_msgs/msg/Float64MultiArray` | 重置连续编码器恢复基准 |

### 3.3 底层控制话题

| 话题 | 类型 | 对应接口/对象 |
|---|---|---|
| `/cia402_cmd_controller/commands` | `std_msgs/msg/Float64MultiArray` | Control Word，`0x6040` |
| `/cia402_mode_controller/commands` | `std_msgs/msg/Float64MultiArray` | Modes of Operation，`0x6060` |
| `/fault_reset_controller/commands` | `std_msgs/msg/Float64MultiArray` | `reset_fault` command interface |
| `/pp_controller/commands` | `std_msgs/msg/Float64MultiArray` | `0x607A` 原始位置 counts |
| `/csp_controller/commands` | `std_msgs/msg/Float64MultiArray` | `0x607A` 原始位置 counts |
| `/csv_controller/commands` | `std_msgs/msg/Float64MultiArray` | `0x60FF` 原始速度值 |
| `/pv_controller/commands` | `std_msgs/msg/Float64MultiArray` | `0x60FF` 原始速度值 |

### 3.4 Controller Manager 服务

| 服务 | 类型 | 用途 |
|---|---|---|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | 读取控制器状态 |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | 切换运动控制器 |

项目内 publisher、subscriber 和 service client 的 QoS depth 均为 10。

## 4. 状态读取

### 4.1 `/multi_motor/states_deg`

这是 `unit_converter` 发布的物理单位状态。

消息类型：

```text
control_msgs/msg/DynamicJointState
```

每个关节的 `interface_names` 包含：

| 字段 | 含义 |
|---|---|
| `position_mm` / `position_deg` | 原始驱动位置经过机械参数和零点换算后的物理位置 |
| `velocity_mm_s` / `velocity_deg_s` | 物理速度 |
| `raw_position_cnt` | 驱动器 `0x6064` 原始计数 |
| `restored_position_cnt` | 恢复后的连续编码器计数 |
| `restored_position_mm` / `restored_position_deg` | 恢复后的连续物理位置 |
| `restore_offset_cnt` | 连续计数与原始计数的偏移 |
| `turn` | 连续位置中的整数圈数 |
| `single_cnt` | 当前单圈内计数 |

解析时通过 `joint_names` 确定关节，通过每个 `InterfaceValue.interface_names` 确定字段。

当编码器恢复尚未准备好时，以下字段为 `NaN`：

```text
restored_position_cnt
restored_position_mm / restored_position_deg
restore_offset_cnt
turn
single_cnt
```

### 4.2 `/dynamic_joint_states`

该话题由 `joint_state_broadcaster` 发布。

当前代码读取：

```text
status_word
modes_of_operation_display
```

对应 EtherCAT 对象：

| 字段 | 对象 |
|---|---|
| `status_word` | `0x6041` |
| `modes_of_operation_display` | `0x6061` |

CiA 402 状态解析位于：

```python
from multi_motor_control.cia402 import parse_status_word

drive_status = parse_status_word(status_word)
```

`DriveStatus` 字段：

```text
raw
state
ready_to_switch_on
switched_on
operation_enabled
fault
voltage_enabled
quick_stop
switch_on_disabled
warning
target_reached
internal_limit
```

当前状态名称与掩码：

| 状态 | value | mask |
|---|---:|---:|
| Not Ready To Switch On | `0x0000` | `0x004F` |
| Switch On Disabled | `0x0040` | `0x004F` |
| Ready To Switch On | `0x0021` | `0x006F` |
| Switched On | `0x0023` | `0x006F` |
| Operation Enabled | `0x0027` | `0x006F` |
| Quick Stop Active | `0x0007` | `0x006F` |
| Fault Reaction Active | `0x000F` | `0x004F` |
| Fault | `0x0008` | `0x004F` |

`DriveStatus.quick_stop` 当前直接反映 Status Word bit 5。该值为 `True` 时表示 Quick Stop 条件未激活。

### 4.3 `/joint_states_raw`

该话题是 `ros2_control_node` 的 `/joint_states` 经 launch 重映射后的原始反馈：

```text
position = 0x6064 原始 counts
velocity = 0x606C 原始 counts/s
```

`unit_converter` 订阅该话题。

### 4.4 `/joint_states`

该话题由 `unit_converter` 重新发布：

```text
position = 原始 0x6064 counts
velocity = 原始 0x606C counts/s
effort   = restored continuous counts
```

当前 `effort` 字段不是力或转矩。

### 4.5 控制器状态

调用：

```text
/controller_manager/list_controllers
```

GUI 每 1 秒调用一次该服务，并读取：

```text
pp_controller
csp_controller
csv_controller
pv_controller
```

常见状态：

```text
active
inactive
not loaded
```

## 5. CiA 402 控制字

控制话题：

```text
/cia402_cmd_controller/commands
std_msgs/msg/Float64MultiArray
```

数组中的浮点值在硬件接口中转换为 `uint16` Control Word。

`cia402.py` 当前定义：

| 常量 | 值 | 含义 |
|---|---:|---|
| `CW.DISABLE_VOLTAGE` | `0x0000` | Disable Voltage |
| `CW.QUICK_STOP` | `0x0002` | Quick Stop |
| `CW.SHUTDOWN` | `0x0006` | Shutdown |
| `CW.SWITCH_ON` | `0x0007` | Switch On |
| `CW.ENABLE_OPERATION` | `0x000F` | Enable Operation |
| `CW.FAULT_RESET` | `0x0080` | Fault Reset |
| `CW.HALT_BIT` | `0x0100` | Halt |
| `CW.NEW_SET_POINT` | `0x0010` | PP New Set-point |
| `CW.CHANGE_IMMEDIATE` | `0x0020` | PP Change Immediately |

### 5.1 `next_control_word()`

现有函数：

```python
next_control_word(current_sw: int) -> int
```

行为：

```text
Fault                  -> 0x0080
Switch On Disabled     -> 0x0006
Ready To Switch On     -> 0x0007
Switched On            -> 0x000F
Operation Enabled      -> 0x000F
其他状态               -> 0x000F
```

GUI 的 Enable 流程每 50 ms 读取一次 `status_word`，调用该函数并发布下一步控制字。

Enable 最多运行 60 个 tick，即约 3 秒。

### 5.2 其他关节的 Control Word

GUI 构造五轴控制字向量时，根据每个关节当前状态填入：

```text
Operation Enabled -> 0x000F
Switched On       -> 0x0007
Ready To Switch On -> 0x0006
其他               -> 0x0000
```

因此操作一个关节时，其他关节仍保留与其当前状态匹配的控制字。

## 6. 运行模式

模式话题：

```text
/cia402_mode_controller/commands
std_msgs/msg/Float64MultiArray
```

对应 `0x6060 Modes of Operation`。

### 6.1 当前可用模式

| 模式 | 值 | 控制器 | 命令对象 |
|---|---:|---|---|
| PP | 1 | `pp_controller` | `0x607A` |
| CSP | 8 | `csp_controller` | `0x607A` |
| CSV | 9 | `csv_controller` | `0x60FF` |
| PV | 3 | `pv_controller` | `0x60FF` |

`cia402.py` 还定义：

```text
0  = No Mode
4  = Profile Torque
6  = Homing
7  = Interpolated Position
10 = CST
```

但当前 `controllers.yaml` 没有为这些模式配置运动控制器。

### 6.2 控制器互斥

四个运动控制器都配置了五个关节，并占用相同类型的命令接口。

GUI 切换运动控制器时调用：

```python
RosBridge.switch_controller(
    activate=[target_controller],
    deactivate=[
        controller
        for controller in (
            "pp_controller",
            "csp_controller",
            "csv_controller",
            "pv_controller",
        )
        if controller != target_controller
    ],
    strictness=SwitchController.Request.BEST_EFFORT,
)
```

所以当前同一时刻只能有一个运动控制器为 active。

## 7. 物理单位命令

物理单位命令先进入 `unit_converter`，再转换为底层 counts。

### 7.1 输入向量检查

`unit_converter._validate_vector()` 要求：

```text
len(msg.data) == num_joints
```

当前 `num_joints = 5`。

长度不一致时：

- 记录 warning；
- 丢弃整帧；
- 不向底层控制器发布。

### 7.2 PP 位置命令

```text
/multi_motor/pp_position_deg/commands
std_msgs/msg/Float64MultiArray
```

单位：

```text
joint_1     = mm
joint_2..5  = deg
```

`unit_converter` 处理：

1. 检查数组长度；
2. 加载当前位置限制；
3. 对每个关节执行 clamp；
4. 将标定坐标转换为 restored target counts；
5. 减去 `restore_offset_cnt`；
6. 发布到 `/pp_controller/commands`。

任意关节的 encoder restore 未就绪时，整帧位置命令被丢弃。

GUI 发送 PP 命令后，对目标关节发布：

```text
0x003F = 0x000F | 0x0010 | 0x0020
```

约 100 ms 后恢复：

```text
0x000F
```

该上升沿触发 Denali XCR 接受新的 PP 目标。

### 7.3 CSP 位置命令

```text
/multi_motor/csp_position_deg/commands
std_msgs/msg/Float64MultiArray
```

单位：

```text
joint_1     = mm
joint_2..5  = deg
```

转换和拒绝规则与 PP 相同。

GUI 的 CSP timer 为：

```text
100 Hz
```

每次发布完整五轴向量：

- 正在 streaming 的关节使用 GUI 目标；
- 未 streaming 的关节使用其当前反馈位置。

### 7.4 CSV 速度命令

```text
/multi_motor/csv_velocity_deg_s/commands
std_msgs/msg/Float64MultiArray
```

单位：

```text
joint_1     = mm/s
joint_2..5  = deg/s
```

转换后发布到：

```text
/csv_controller/commands
```

### 7.5 PV 速度命令

```text
/multi_motor/pv_velocity_deg_s/commands
std_msgs/msg/Float64MultiArray
```

单位与 CSV 相同。

转换后发布到：

```text
/pv_controller/commands
```

当前速度命令只执行单位换算和 int32 范围保护，不读取 `calibrate.json` 中的位置范围限制速度。

## 8. 当前 GUI 的控制流程

本节描述 `my_motor_rqt_plugin.py` 当前实际执行的流程。

### 8.1 Enable

GUI 中对应：

```python
_start_enable_sequence()
_walk_to_operation_enabled()
_activate_motion_controller()
```

流程：

1. 读取 GUI 中选择的目标模式；
2. 如果目标模式是 PP 或 CSP，先使用 CSV 作为使能模式；
3. CSV 目标速度清零；
4. 激活 `csv_controller`；
5. 发布 Modes of Operation；
6. PP/CSP 目标位置对齐到当前 restored feedback；
7. 等待 300 ms；
8. 每 50 ms 调用 `next_control_word()`；
9. 到达 Operation Enabled 后激活目标运动控制器；
10. PP/CSP 再执行位置预写入和模式注入。

### 8.2 Enable All

GUI 中对应：

```python
_start_enable_all_staggered()
```

当前参数：

```text
STAGGER_BATCH_SIZE = 1
STAGGER_DELAY_MS   = 600
```

每次使能一个关节，相邻关节启动间隔 600 ms。

已经 Operation Enabled 的关节会跳过；Fault 状态的关节不会自动使能。

### 8.3 Disable 单轴

GUI 中对应：

```python
_on_disable(idx)
```

行为：

- 停止该关节正在执行的 Enable timer；
- 对目标关节写入 `CW.DISABLE_VOLTAGE = 0x0000`；
- 其他关节保持其当前状态对应的 Control Word。

### 8.4 Disable All

GUI 中对应：

```python
_disable_all()
```

行为：

1. 停止标定；
2. 取消所有 Enable timer；
3. 停止 CSP streaming；
4. 向 CSV 发布五轴全零；
5. 向 PV 发布五轴全零；
6. 向五个关节写入 `0x0000`。

### 8.5 Fault Reset

GUI 中对应：

```python
_on_reset_fault(idx)
```

流程：

1. `/fault_reset_controller/commands` 中目标关节写 1；
2. 100 ms 后写回 0；
3. `/cia402_cmd_controller/commands` 中目标关节写 `0x0080`；
4. 等待 200 ms；
5. 调用 `_start_enable_sequence()` 重新使能。

### 8.6 Apply PP

GUI 中对应：

```python
_on_apply_mode(idx)
```

PP 流程：

1. GUI 目标位置对齐到当前 feedback；
2. 激活 `pp_controller`，停用其他运动控制器；
3. 50 ms 后向 PP 物理单位话题发布当前五轴位置；
4. 等待 200 ms；
5. 再次发布最新位置；
6. 向目标关节写入 PP mode = 1。

### 8.7 Apply CSP

CSP 流程：

1. GUI 目标位置对齐到当前 feedback；
2. 激活 `csp_controller`；
3. 50 ms 后发布当前五轴位置；
4. 等待 200 ms；
5. 再次发布最新位置；
6. 向目标关节写入 CSP mode = 8。

模式写入在位置命令接口稳定之后执行。

### 8.8 Apply CSV/PV

CSV/PV 流程：

1. 写入目标 Modes of Operation；
2. 等待 200 ms；
3. 激活对应控制器，停用其他运动控制器。

### 8.9 PP Send

GUI 中对应：

```python
_on_send_pp(idx)
```

只有 `_active_ctrl == "pp_controller"` 时执行。

发送内容：

- 目标关节使用 GUI 设定位置；
- 其他关节使用当前反馈位置；
- 发布完整五轴位置；
- 发送 PP New Set-point Control Word 上升沿。

### 8.10 CSP Start/Stop

GUI 中对应：

```python
_on_stream_start(idx)
_on_stream_stop(idx)
_tick_csp_stream()
```

Start：

- 检查 active controller 是否为 `csp_controller`；
- 将首帧目标强制对齐当前 feedback；
- 将该关节标记为 streaming；
- 启动 100 Hz timer。

Stop：

- 清除该关节 streaming 标记；
- 所有关节停止 streaming 后关闭 timer。

### 8.11 CSV/PV Send

GUI 中对应：

```python
_on_send_vel(idx)
```

当 active controller 为：

- `csv_controller`：更新 `_csv_targets[idx]` 并发布完整向量；
- `pv_controller`：更新 `_pv_targets[idx]` 并发布完整向量。

其他模式下不发送速度。

### 8.12 Go Default

GUI 中对应：

```python
_go_default()
```

行为：

- 只控制 `joint_2`～`joint_5`；
- `joint_1` 保持当前位置；
- 读取 `calibrate.json` 中的 `default_deg`；
- 将 restored position 转换为驱动 counts；
- 先发布当前 raw counts；
- 激活 `csp_controller`；
- 250 ms 后发布默认位置 counts 并写入 CSP mode；
- 400 ms 后再次发布目标 counts。

## 9. 当前 Python 函数

### 9.1 `cia402.py`

```python
parse_status_word(sw: int) -> DriveStatus
```

用途：解析 16 位 Status Word。

```python
next_control_word(current_sw: int) -> int
```

用途：根据当前状态返回走向 Operation Enabled 的下一步 Control Word。

### 9.2 `RosBridge`

类位置：

```text
multi_motor_control/my_motor_rqt_plugin.py
```

当前 publish 函数：

```python
publish_control_word(vec: list[float])
publish_mode(vec: list[float])
publish_fault_reset(vec: list[float])
publish_pp(vec: list[float])
publish_csp(vec: list[float])
publish_csp_counts(vec: list[float])
publish_csv(vec: list[float])
publish_pv(vec: list[float])
publish_reset_encoder_restore(vec: list[float])
```

这些函数统一调用：

```python
_publish_array(pub, vec)
```

并将列表转换为：

```text
std_msgs/msg/Float64MultiArray
```

控制器切换函数：

```python
switch_controller(
    activate: list[str],
    deactivate: list[str],
    strictness: int = SwitchController.Request.BEST_EFFORT,
)
```

函数设置：

```text
activate_controllers
deactivate_controllers
strictness
activate_asap = True
```

完成后读取响应中的 `ok`。

### 9.3 `joint_limits.py`

加载函数：

```python
load_joint_geometry(
    explicit_path: str = "",
) -> tuple[dict[str, dict], Path | None]
```

```python
load_joint_limits(
    explicit_path: str = "",
    geometry_path: str = "",
) -> tuple[dict[str, JointLimit], Path | None]
```

保存函数：

```python
save_min_zero_calibration(
    path: Path,
    joint_names,
    raw_min_by_joint: dict[str, float],
) -> dict[str, JointLimit]
```

```python
save_range_calibration(
    path: Path,
    joint_names,
    raw_min_by_joint: dict[str, float],
    raw_max_by_joint: dict[str, float],
) -> dict[str, JointLimit]
```

```python
save_linear_manual_calibration(
    path: Path,
    joint_name: str,
    zero_position_cnt: float,
    range_mm: float,
    encoder_counts_per_rev: float = 131072.0,
    screw_lead_mm_per_rev: float = 10.0,
    linear_direction: float = 1.0,
) -> dict[str, JointLimit]
```

### 9.4 `JointLimit`

当前主要方法：

```python
clamp(value_deg: float) -> float
raw_to_calibrated(raw_deg: float) -> float
calibrated_to_raw(calibrated_deg: float) -> float
counts_to_raw_deg(raw_counts, fallback_encoder_resolution=None) -> float
raw_counts_to_calibrated(
    raw_counts,
    raw_deg=None,
    fallback_encoder_resolution=None,
) -> float
calibrated_to_raw_counts(
    calibrated_value,
    encoder_resolution=None,
) -> float
```

属性：

```python
is_linear -> bool
display_unit -> str
```

## 10. 单位换算

### 10.1 旋转轴

静态参数：

```text
motor encoder = 131072 counts/rev
gear ratio    = 6.6
output        = 865075.2 counts/rev
```

位置换算：

```text
counts = deg × encoder_resolution / 360
deg    = counts × 360 / encoder_resolution
```

### 10.2 直线轴

当前 `joint_1` 参数：

```text
encoder_counts_per_rev = 131072
screw_lead_mm_per_rev  = 10
linear_direction       = 1
```

位置换算：

```text
mm = (counts - zero_position_cnt)
     × linear_direction
     × screw_lead_mm_per_rev
     / encoder_counts_per_rev
```

### 10.3 位置 clamp

`unit_converter._limit_position_deg()` 对所有 PP/CSP 位置命令执行：

```text
limited = min(max(command, min), max)
```

超限时记录 warning，并使用 clamp 后的值继续发布。

### 10.4 restored position 到驱动 counts

PP/CSP 目标使用：

```text
restored_target_cnt = calibrated position 转换后的连续目标计数
drive_target_cnt    = restored_target_cnt - restore_offset_cnt
```

restored encoder state 未就绪时返回 `None`，整帧位置命令不发布。

## 11. 标定功能

### 11.1 M1 直线轴

GUI 按钮：

```text
Calibrate Linear M1
Save Linear M1
```

流程：

1. 进入记录状态；
2. 人工将 M1 移动到行程两端；
3. 持续记录最小/最大 raw counts；
4. 保存时计算行程；
5. 写入 `zero_position_cnt`、`range_mm`、`min_mm`、`max_mm`。

### 11.2 M2～M5 零点

GUI 按钮：

```text
Calibrate Rotary Zero M2-M5
```

当前常量：

```text
CALIBRATION_VELOCITY_DEG_S         = 8.0
CALIBRATION_STALL_VEL_DEG_S       = 1.0
CALIBRATION_STALL_SECONDS         = 1.5
CALIBRATION_TARGET_LEAD_DEG       = 3.0
CALIBRATION_FAULT_TARGET_LEAD_DEG = 0.5
CALIBRATION_STALL_PROGRESS_DEG    = 0.2
CALIBRATION_MIN_RUN_SECONDS       = 1.0
CALIBRATION_MIN_MOVEMENT_DEG      = 1.0
CALIBRATION_NO_MOVE_ABORT_SECONDS = 3.0
CALIBRATION_TIMEOUT_SECONDS       = 45.0
```

流程使用原始 CSP counts 绕过尚未建立的物理单位限位，向负方向搜索端点。检测到堵转后保存：

```text
min_deg = 0
max_deg = range_deg
home_offset_deg = -raw_min_deg
```

随后通过 `/multi_motor/reset_encoder_restore` 清除该关节恢复偏移。

### 11.3 M2～M5 行程

GUI 按钮：

```text
Calibrate Rotary Range M2-M5
```

流程：

1. 向负方向搜索最小端点；
2. 向正方向搜索最大端点；
3. 计算 `range_deg = raw_max - raw_min`；
4. 更新 `min_deg = 0` 和 `max_deg = range_deg`；
5. 保留已有 `home_offset_deg`。

完整行程标定超时：

```text
CALIBRATION_RANGE_TIMEOUT_SECONDS = 240
```

这些端点由目标超前、实际速度和位置停滞推断。

## 12. 连续编码器恢复

`unit_converter` 维护每个关节的：

```text
raw_6064_cnt
restore_offset_cnt
continuous_cnt
turn
single_cnt
valid
```

默认状态文件：

```text
<workspace>/state/multi_motor_encoder_state.json
```

### 12.1 保存条件

状态保存需要：

- 所有关节 state 均 valid；
- 所有关节反馈均 ready；
- 所有关节原始 counts 非零；
- 当前不处于 save holdoff；
- 距离上次保存达到最短周期；
- 至少一个关节变化达到最小 counts。

### 12.2 状态字保护

默认：

```text
encoder_require_status_word = true
encoder_status_timeout_sec  = 0.05
```

当状态字为零、缺失或超过 timeout：

- 不接受新的原始位置；
- 保留上次可信连续位置；
- 抑制状态文件写入；
- 将该关节标记为需要重新稳定。

### 12.3 上电稳定

默认：

```text
encoder_powerup_stabilize_sec     = 0.2
encoder_raw_stable_duration_sec   = 0.1
encoder_raw_stable_delta_counts   = 128
```

反馈恢复后，原始位置必须在规定范围内保持稳定，才重新更新连续位置。

### 12.4 跳变重对齐

默认阈值：

```text
encoder_realign_threshold_counts = 65536
```

连续两次可信原始位置差值超过阈值时：

- 保留跳变前的连续位置；
- 等待原始位置稳定；
- 计算最接近旧连续位置的整数圈偏移；
- 重对齐 `restore_offset_cnt`；
- 延迟写入状态文件。

### 12.5 手动重置恢复基准

话题：

```text
/multi_motor/reset_encoder_restore
std_msgs/msg/Float64MultiArray
```

消息必须为完整五轴向量：

- 需要重置的关节填写当前 raw counts；
- 不处理的关节填写 `NaN`。

例如只重置 `joint_2`：

```text
[NaN, joint_2_raw_counts, NaN, NaN, NaN]
```

节点会：

- 将该 raw counts 设为连续位置基准；
- 清除该关节恢复偏移；
- 立即重写 encoder state 文件。

## 13. Launch 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `num_joints` | `5` | 关节数量 |
| `master_id` | `0` | IgH Master 编号 |
| `urdf_file` | `multi_motor.urdf.xacro` | URDF xacro |
| `controllers_file` | `controllers.yaml` | 控制器配置 |
| `slave_config_file` | `ethercat_system.yaml` | EtherCAT 从站配置 |
| `encoder_resolution` | `865075.2` | 默认旋转轴每圈 counts |
| `position_limits_file` | 空，自动查找 | 标定 JSON 路径 |
| `joint_geometry_file` | 空，自动查找 | 机械参数 YAML 路径 |
| `encoder_state_file` | 空，自动生成 | 编码器状态文件 |
| `encoder_one_turn_cnt` | `131072` | 单圈 counts |
| `encoder_state_save_period_sec` | `0.02` | 最短保存周期 |
| `encoder_state_min_delta_counts` | `1` | 最小保存变化 |
| `encoder_realign_threshold_counts` | `65536` | 跳变重对齐阈值 |
| `encoder_require_status_word` | `true` | 是否要求有效状态字 |
| `encoder_status_timeout_sec` | `0.05` | 状态字超时 |
| `encoder_realign_save_holdoff_sec` | `0.5` | 重对齐后保存延迟 |
| `encoder_powerup_stabilize_sec` | `0.2` | 上电稳定等待 |
| `encoder_raw_stable_duration_sec` | `0.1` | 原始位置稳定时间 |
| `encoder_raw_stable_delta_counts` | `128` | 原始位置稳定范围 |

查看当前 launch 参数：

```bash
ros2 launch multi_motor_control multi_motor_control.launch.py --show-args
```

## 14. 配置文件

### 14.1 `controllers.yaml`

包含：

- controller manager 1000 Hz update rate；
- 五轴关节列表；
- always-on 控制器；
- PP/CSP/CSV/PV 控制器类型。

### 14.2 `ethercat_system.yaml`

当前 Denali XCR 参数：

```text
vendor_id      = 0x0000029c
product_id     = 0x03831002
assign_activate = 0x0300
auto_fault_reset = false
auto_state_transitions = false
```

RxPDO：

```text
0x6040 Control Word
0x607A Target Position
0x60FF Target Velocity
0x6060 Modes of Operation
```

TxPDO：

```text
0x6041 Status Word
0x6064 Position Actual Value
0x606C Velocity Actual Value
0x6061 Modes of Operation Display
```

### 14.3 `joint_geometry.yaml`

保存不会随每次标定变化的机械参数：

- 轴类型；
- 显示单位；
- 编码器单圈 counts；
- 旋转轴输出端分辨率；
- 丝杠导程；
- 直线方向。

### 14.4 `calibrate.json`

保存实机标定参数：

- 软件最小/最大位置；
- 有效行程；
- 默认位置；
- 旋转轴零点偏移；
- M1 零点 counts。

`unit_converter` 每 1 秒重新加载该文件。

## 15. 当前实现限制

- 命令向量必须完整包含所有关节；
- 同一时刻只有一个运动控制器 active；
- 当前控制器按五轴整体配置，不支持位置模式和速度模式按关节混用；
- PP/CSP 位置命令在任意关节 encoder restore 未就绪时整帧丢弃；
- PP/CSP 位置命令会 clamp，速度命令没有独立的软件速度限位；
- `/joint_states.effort` 当前存放连续编码器 counts，不是转矩；
- `calibrate.json` 为当前实机数据；
- 旋转标定使用堵转特征判断端点；
- URDF 中的几何模型为占位模型，不代表真实末端执行器运动学；
- 编码器和计算机同时断电期间发生的机械运动无法通过当前恢复算法推断。
