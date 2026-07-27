# 末端执行器控制 API 参考

本文面向负责上层整机控制 API 的开发人员，说明 `MPKEthercat_ws` 当前能够提供的状态读取、驱动控制、运动命令、配置选项与调用约束。

本文以当前五轴代码为准。实际运行参数的最终来源是：

- `src/multi_motor/config/controllers.yaml`
- `src/multi_motor/config/ethercat_system.yaml`
- `src/multi_motor/config/joint_geometry.yaml`
- `src/multi_motor/calibrate.json`
- `src/multi_motor/launch/multi_motor_control.launch.py`

## 1. API 边界

当前项目已经提供：

- ROS 2 状态话题；
- ROS 2 运动命令话题；
- CiA 402 控制字和运行模式话题；
- `controller_manager` 控制器查询与切换服务；
- 单位转换、位置限位和连续编码器恢复节点；
- rqt/Qt GUI 中已经验证过的使能、模式切换和标定流程。

当前项目尚未提供统一的高层服务，例如：

```text
enable()
disable()
set_mode()
move_position()
move_velocity()
get_state()
```

整机控制 API 应当封装本文所列 ROS 2 接口与时序，不建议让整机业务直接操作 `0x6040` 控制字或底层 counts 话题。

## 2. 系统启动

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 launch multi_motor_control multi_motor_control.launch.py
```

启动后会创建：

- EtherCAT `ros2_control` 硬件接口；
- `joint_state_broadcaster`；
- CiA 402 控制字、模式和故障复位控制器；
- PP、CSP、CSV、PV 四个运动控制器；
- `unit_converter`；
- `robot_state_publisher`。

四个运动控制器启动时均为 `inactive`，所有电机不会自动使能。

上层 API 在允许控制前至少应确认：

1. `/dynamic_joint_states` 正在持续更新；
2. 五个关节均已出现；
3. 每个关节的 `status_word` 非零；
4. `/multi_motor/states_deg` 中的 restored position 为有限值；
5. `/controller_manager/list_controllers` 可用；
6. 目标运动控制器已经加载。

## 3. 关节模型、顺序与单位

所有数组型命令必须严格包含 5 个元素，顺序固定为：

```text
[joint_1, joint_2, joint_3, joint_4, joint_5]
```

不支持只发送一个元素控制单轴。控制单轴时，仍需构造完整五轴向量。

| 关节 | 机构类型 | 位置单位 | 速度单位 | 当前软件范围 | 当前默认位置 |
|---|---|---|---|---:|---:|
| `joint_1` | 滚珠丝杠直线轴 | mm | mm/s | 0 ～ 264.8367 mm | 0 mm |
| `joint_2` | 旋转轴 | deg | deg/s | 0 ～ 353.2292 deg | 266.3473 deg |
| `joint_3` | 旋转轴 | deg | deg/s | 0 ～ 353.2400 deg | 60.6616 deg |
| `joint_4` | 旋转轴 | deg | deg/s | 0 ～ 352.6212 deg | 139.9350 deg |
| `joint_5` | 旋转轴 | deg | deg/s | 0 ～ 353.5658 deg | 23.9044 deg |

以上范围来自当前 `calibrate.json`，重新标定后会变化。上层 API 不应写死这些数值，应在启动时读取标定文件或由参数服务器/配置管理模块提供。

虽然部分话题名包含 `_deg`，但 `joint_1` 在这些话题中实际使用 mm 或 mm/s。

## 4. 读取接口

### 4.1 推荐状态接口：`/multi_motor/states_deg`

```text
Topic: /multi_motor/states_deg
Type:  control_msgs/msg/DynamicJointState
QoS:   depth 10
```

这是上层控制最推荐使用的反馈接口。

每个关节的 `interface_names` 如下：

| 字段 | 含义 |
|---|---|
| `position_mm` / `position_deg` | 当前原始驱动位置经过机械零点换算后的物理位置 |
| `velocity_mm_s` / `velocity_deg_s` | 当前物理速度 |
| `raw_position_cnt` | 驱动对象 `0x6064` 的原始计数 |
| `restored_position_cnt` | 经过多圈恢复后的连续计数 |
| `restored_position_mm` / `restored_position_deg` | 经过多圈恢复与标定换算后的连续物理位置 |
| `restore_offset_cnt` | 连续计数与原始计数之间的恢复偏移 |
| `turn` | 连续计数对应的整数圈数 |
| `single_cnt` | 当前圈内计数 |

上层 API 应优先使用：

```text
restored_position_mm
restored_position_deg
velocity_mm_s
velocity_deg_s
```

如果 restored position 为 `NaN`，表示编码器恢复状态尚未准备好。此时不得发送 PP/CSP 位置命令。

解析时应按 `joint_names` 查找关节，按 `interface_names` 查找字段，不应假定消息内部字段顺序永远不变。

### 4.2 CiA 402 状态接口：`/dynamic_joint_states`

```text
Topic: /dynamic_joint_states
Type:  control_msgs/msg/DynamicJointState
QoS:   depth 10
```

主要读取：

| 字段 | 含义 |
|---|---|
| `status_word` | CiA 402 Status Word，驱动对象 `0x6041` |
| `modes_of_operation_display` | 当前实际运行模式，驱动对象 `0x6061` |

`status_word` 的常用状态：

| 状态 | 典型掩码结果 |
|---|---:|
| Not Ready To Switch On | `0x0000` |
| Switch On Disabled | `0x0040` |
| Ready To Switch On | `0x0021` |
| Switched On | `0x0023` |
| Operation Enabled | `0x0027` |
| Quick Stop Active | `0x0007` |
| Fault Reaction Active | `0x000F` |
| Fault | `0x0008` |

项目已提供：

```python
from multi_motor_control.cia402 import parse_status_word

status = parse_status_word(status_word)
```

返回的 `DriveStatus` 包含：

```text
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

注意：当前 `DriveStatus.quick_stop` 直接反映 Status Word bit 5；该位为 `True` 表示 Quick Stop 条件未激活。若上层 API 希望提供更直观的字段，建议转换为 `quick_stop_active = not status.quick_stop`。

### 4.3 原始与恢复后的 JointState

```text
/joint_states_raw   sensor_msgs/msg/JointState
/joint_states       sensor_msgs/msg/JointState
```

`/joint_states_raw`：

- `position`：原始 `0x6064` counts；
- `velocity`：原始 `0x606C` counts/s。

`/joint_states`：

- `position`：仍为原始 counts；
- `velocity`：仍为原始 counts/s；
- `effort`：当前实现中存放 restored continuous counts，不是力矩。

因此整机 API 不应把 `/joint_states.effort` 解释为关节力或转矩。

### 4.4 控制器状态服务

```text
Service: /controller_manager/list_controllers
Type:    controller_manager_msgs/srv/ListControllers
```

重点检查：

```text
pp_controller
csp_controller
csv_controller
pv_controller
```

其状态可能为：

```text
unconfigured
inactive
active
```

## 5. 控制模式与可选项

当前对外支持四种运动模式：

| API 枚举建议 | CiA 402 值 | 控制器 | 命令类型 | 使用场景 |
|---|---:|---|---|---|
| `PP` | 1 | `pp_controller` | 单次目标位置 | 点到点位置运动 |
| `CSP` | 8 | `csp_controller` | 周期目标位置 | 连续轨迹跟踪 |
| `CSV` | 9 | `csv_controller` | 周期目标速度 | 连续速度控制 |
| `PV` | 3 | `pv_controller` | Profile Velocity | 单次/低频速度设定 |

代码中还定义了 Profile Torque、Homing、Interpolated Position、CST 等 CiA 402 常量，但当前没有对应的 `ros2_control` 控制器和公开命令接口，不应暴露为可用模式。

### 5.1 重要限制：模式是系统级的

当前四个运动控制器都会占用五个关节，并且必须互斥。

因此当前架构适合：

```text
五个关节统一处于 PP
五个关节统一处于 CSP
五个关节统一处于 CSV
五个关节统一处于 PV
```

不适合：

```text
joint_1 使用 CSV，同时 joint_2 使用 CSP
```

上层 API 建议将 `set_mode(mode)` 设计为系统级函数，而不是单关节函数。

如果整机确实需要不同关节同时运行于不同类型模式，需要重新拆分 `controllers.yaml`，为各关节或关节组创建独立控制器。

## 6. 物理单位运动命令

所有以下话题均使用：

```text
std_msgs/msg/Float64MultiArray
```

且数组长度必须等于关节数。当前为 5。

### 6.1 PP 位置

```text
Topic: /multi_motor/pp_position_deg/commands
Units: joint_1 = mm, joint_2..5 = deg
```

处理行为：

- 自动应用各关节软件位置范围；
- 超限值会被 clamp 到最近边界；
- 使用 restored position 坐标转换到驱动原始 counts；
- 任意关节的编码器恢复未就绪时，整帧命令被拒绝。

PP 控制不仅需要发布位置，还需要对目标关节产生 New Set-point 上升沿：

```text
0x003F = ENABLE_OPERATION | NEW_SET_POINT | CHANGE_IMMEDIATE
等待约 100 ms
0x000F = ENABLE_OPERATION
```

### 6.2 CSP 位置

```text
Topic: /multi_motor/csp_position_deg/commands
Units: joint_1 = mm, joint_2..5 = deg
```

GUI 当前以 100 Hz 发布 CSP 命令。

推荐行为：

- 第一帧必须使用当前 restored position；
- 未运动关节必须填入当前保持位置；
- 轨迹执行过程中持续发布完整五轴向量；
- 停止轨迹时先保持当前位置，再决定是否切换模式或 Disable。

### 6.3 CSV 速度

```text
Topic: /multi_motor/csv_velocity_deg_s/commands
Units: joint_1 = mm/s, joint_2..5 = deg/s
```

### 6.4 PV 速度

```text
Topic: /multi_motor/pv_velocity_deg_s/commands
Units: joint_1 = mm/s, joint_2..5 = deg/s
```

速度接口当前只进行单位换算和 int32 范围保护，不使用 `calibrate.json` 限制最大速度。上层 API 必须自行定义并检查：

- 每轴最大速度；
- 每轴最大加速度；
- 命令超时；
- 通信中断后的归零策略。

对于只控制一个关节的速度命令，建议其他关节填 0，而不是沿用不确定的旧速度。

## 7. 底层控制接口

以下接口用于实现高层封装，不建议直接暴露给整机业务。

### 7.1 Control Word

```text
Topic: /cia402_cmd_controller/commands
Type:  std_msgs/msg/Float64MultiArray
```

常用值：

| 名称 | 值 | 说明 |
|---|---:|---|
| `DISABLE_VOLTAGE` | `0x0000` | 关闭驱动输出 |
| `QUICK_STOP` | `0x0002` | Quick Stop |
| `SHUTDOWN` | `0x0006` | 进入 Ready To Switch On |
| `SWITCH_ON` | `0x0007` | 进入 Switched On |
| `ENABLE_OPERATION` | `0x000F` | 进入/保持 Operation Enabled |
| `FAULT_RESET` | `0x0080` | Fault Reset 上升沿 |
| `HALT_BIT` | `0x0100` | Halt 位 |
| `NEW_SET_POINT` | `0x0010` | PP 新目标上升沿 |
| `CHANGE_IMMEDIATE` | `0x0020` | PP 立即应用目标 |

### 7.2 Modes of Operation

```text
Topic: /cia402_mode_controller/commands
Type:  std_msgs/msg/Float64MultiArray
```

当前可用值：

```text
1 = PP
3 = PV
8 = CSP
9 = CSV
```

模式是否真正生效，应通过 `/dynamic_joint_states` 中的 `modes_of_operation_display` 确认，不能仅以“命令已发布”作为成功依据。

### 7.3 Fault Reset

```text
Topic: /fault_reset_controller/commands
Type:  std_msgs/msg/Float64MultiArray
```

GUI 当前使用双重复位：

1. 对目标关节发布 `reset_fault = 1`；
2. 约 100 ms 后恢复 `reset_fault = 0`；
3. 同时在 Control Word 中对目标关节发布 `0x0080`；
4. 等待约 200 ms；
5. 重新执行模式注入和 CiA 402 Enable 流程。

### 7.4 切换运动控制器

```text
Service: /controller_manager/switch_controller
Type:    controller_manager_msgs/srv/SwitchController
```

推荐请求：

```python
request.activate_controllers = [target_controller]
request.deactivate_controllers = [
    controller for controller in
    ["pp_controller", "csp_controller", "csv_controller", "pv_controller"]
    if controller != target_controller
]
request.strictness = SwitchController.Request.BEST_EFFORT
request.activate_asap = True
```

必须检查响应中的 `ok`。

## 8. 推荐的高层 API

以下函数目前不是项目内现成的 ROS 服务，而是建议整机 API 按此语义封装。

### 8.1 数据类型建议

```python
class ControlMode(Enum):
    PP = 1
    PV = 3
    CSP = 8
    CSV = 9


@dataclass
class JointState:
    name: str
    position: float
    velocity: float
    position_unit: str
    velocity_unit: str
    raw_position_cnt: int
    restored_position_cnt: int
    restore_ready: bool
    status_word: int
    cia402_state: str
    mode: ControlMode | None
    operation_enabled: bool
    fault: bool
    warning: bool
    target_reached: bool
    internal_limit: bool


@dataclass
class SystemState:
    stamp: float
    joints: dict[str, JointState]
    active_controller: str | None
    command_ready: bool
```

### 8.2 读取函数

```python
get_system_state() -> SystemState
get_joint_state(joint: str) -> JointState
get_active_mode() -> ControlMode | None
get_controller_states() -> dict[str, str]
is_ready() -> bool
has_fault(joints: list[str] | None = None) -> bool
```

`is_ready()` 建议同时检查：

- 五轴状态均新鲜；
- 无 `NaN` restored position；
- `status_word` 非零；
- 无 Fault；
- 一个且仅一个目标运动控制器 active，或系统处于未选模式状态。

### 8.3 驱动状态函数

```python
enable(joints: list[str] | None = None,
       mode: ControlMode = ControlMode.CSV,
       timeout: float = 3.0) -> Result

disable(joints: list[str] | None = None) -> Result

disable_all() -> Result

quick_stop(joints: list[str] | None = None) -> Result

reset_fault(joints: list[str] | None = None,
            reenable: bool = False,
            mode: ControlMode = ControlMode.CSV) -> Result
```

建议默认：

- `joints=None` 表示全部关节；
- `enable()` 默认使用 CSV + 零速度；
- `reset_fault()` 默认复位后不自动恢复运动；
- 所有函数返回逐关节结果和最终 `status_word`，不只返回单个布尔值。

### 8.4 模式函数

```python
set_mode(mode: ControlMode,
         align_position: bool = True,
         timeout: float = 2.0) -> Result

get_mode() -> ControlMode | None
```

由于当前控制器是系统级互斥，`set_mode()` 不建议接受单关节参数。

### 8.5 运动函数

```python
move_pp(target: list[float],
        timeout: float | None = None,
        wait_target_reached: bool = False) -> Result

start_csp(initial_target: list[float] | None = None,
          publish_rate_hz: float = 100.0) -> Result

update_csp(target: list[float]) -> Result

stop_csp(hold_position: bool = True) -> Result

set_csv_velocity(velocity: list[float]) -> Result

set_pv_velocity(velocity: list[float]) -> Result

stop_velocity(disable_after_stop: bool = False) -> Result

go_default(rotary_only: bool = True) -> Result
```

所有位置/速度列表都必须为完整五轴向量。

建议 `Result` 至少包含：

```text
success
message
joint_results
requested_values
applied_values
clamped
timestamp
```

当前 `unit_converter` 会在位置超限时 clamp，但不会通过独立响应告诉调用者实际发生了 clamp。若整机 API 需要确定性响应，建议在发布前自行加载 `calibrate.json` 并返回 `applied_values`。

## 9. 推荐控制时序

### 9.1 Enable

推荐安全使能流程：

1. 读取最新反馈；
2. 确认无 Fault；
3. 对速度模式预先发送全零速度；
4. 对 PP/CSP 请求，先用 CSV + 零速度作为使能模式；
5. 发布 Modes of Operation；
6. 根据最新 `status_word` 每 50 ms 发送下一步 Control Word：

```text
Switch On Disabled -> 0x0006
Ready To Switch On -> 0x0007
Switched On        -> 0x000F
Operation Enabled  -> 0x000F
```

7. 每一步都重新读取状态；
8. 到达 Operation Enabled 后再切换目标运动控制器；
9. PP/CSP 必须先把命令位置对齐到当前 restored position；
10. 等控制器命令接口稳定后再写入 PP/CSP mode。

项目已有辅助函数：

```python
from multi_motor_control.cia402 import next_control_word

control_word = next_control_word(status_word)
```

### 9.2 CSP 安全切换

推荐顺序：

1. 当前模式保持 CSV，速度为 0；
2. 激活 `csp_controller`，停用其他运动控制器；
3. 读取五轴当前 restored position；
4. 向 CSP 物理单位话题连续发送当前位置；
5. 等待约 200 ms；
6. 再向 Modes of Operation 写入 CSP = 8；
7. 确认 `modes_of_operation_display == 8`；
8. 从当前位置开始生成连续轨迹。

不得先写入 CSP mode，再发送首帧位置。否则旧的 `0x607A` 目标可能引起位置突跳。

### 9.3 PP 命令

推荐顺序：

1. 激活 `pp_controller`；
2. 发布当前 restored position；
3. 写入 PP mode = 1；
4. 确认 `modes_of_operation_display == 1`；
5. 发布目标位置向量；
6. 对需要运动的关节发布 Control Word `0x003F`；
7. 约 100 ms 后恢复 `0x000F`；
8. 可选：等待 `target_reached`。

### 9.4 CSV/PV 命令

推荐顺序：

1. 先发布全零速度；
2. 写入 CSV = 9 或 PV = 3；
3. 切换并确认对应控制器 active；
4. 确认 Operation Enabled 和 mode display；
5. 发布速度；
6. 停止时先发布全零速度；
7. 确认实际速度接近零后再 Disable。

### 9.5 Stop 与 Disable

建议区分：

```text
stop_motion(): 位置保持或速度归零，驱动仍保持 Operation Enabled
quick_stop():   使用 CiA 402 Quick Stop
disable():      Control Word = 0x0000，关闭驱动输出
```

通信断开、上层心跳超时或轨迹生成器异常时，应自动进入明确的安全策略。当前底层没有替整机 API 实现命令 watchdog。

## 10. 标定与维护接口

标定属于维护功能，不建议混入普通实时运动 API。

已有 Python 辅助函数：

```python
from multi_motor_control.joint_limits import (
    load_joint_geometry,
    load_joint_limits,
    save_linear_manual_calibration,
    save_min_zero_calibration,
    save_range_calibration,
)
```

| 函数 | 用途 |
|---|---|
| `load_joint_geometry()` | 加载机械类型、单位、编码器和丝杠参数 |
| `load_joint_limits()` | 合并机械参数与 `calibrate.json` |
| `save_linear_manual_calibration()` | 保存 M1 手动扫描得到的零点和行程 |
| `save_min_zero_calibration()` | 保存旋转轴最小端点零位 |
| `save_range_calibration()` | 保存旋转轴完整行程 |

GUI 当前维护操作：

```text
Calibrate Linear M1
Calibrate Rotary Zero M2-M5
Calibrate Rotary Range M2-M5
Stop Calibrate
Go Default
```

旋转轴标定通过速度下降和目标超前推断堵转端点，不是安全等级功能。建议整机 API 默认不开放远程自动标定，或要求维护权限、现场确认和急停联锁。

## 11. 编码器恢复接口

```text
Topic: /multi_motor/reset_encoder_restore
Type:  std_msgs/msg/Float64MultiArray
```

该话题不是普通“布尔复位”。

消息必须为五轴向量：

- 需要重置的关节填写当前原始位置 counts；
- 不需要重置的关节填写 `NaN`。

示例：

```text
[NaN, raw_joint_2_counts, NaN, NaN, NaN]
```

节点会立即：

- 将该原始 counts 设为新的连续位置基准；
- 清除恢复偏移；
- 写入 encoder state JSON。

该接口主要供零点标定完成后使用，不建议暴露给普通运动控制调用者。

## 12. Launch 可选参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `num_joints` | `5` | 关节数量 |
| `master_id` | `0` | IgH Master 编号 |
| `urdf_file` | `multi_motor.urdf.xacro` | URDF xacro 文件 |
| `controllers_file` | `controllers.yaml` | 控制器配置 |
| `slave_config_file` | `ethercat_system.yaml` | EtherCAT 从站配置 |
| `encoder_resolution` | `865075.2` | 旋转轴输出端每圈 counts |
| `position_limits_file` | 自动查找 | 自定义 `calibrate.json` |
| `joint_geometry_file` | 自动查找 | 自定义机械参数 YAML |
| `encoder_state_file` | `<workspace>/state/...json` | 连续编码器状态文件 |
| `encoder_one_turn_cnt` | `131072` | 单圈编码器 counts |
| `encoder_state_save_period_sec` | `0.02` | 最短状态保存周期 |
| `encoder_state_min_delta_counts` | `1` | 触发保存的最小变化 |
| `encoder_realign_threshold_counts` | `65536` | 原始计数跳变重对齐阈值 |
| `encoder_require_status_word` | `true` | 要求有效且新鲜的状态字 |
| `encoder_status_timeout_sec` | `0.05` | 状态字最大允许年龄 |
| `encoder_realign_save_holdoff_sec` | `0.5` | 重对齐后的保存延迟 |
| `encoder_powerup_stabilize_sec` | `0.2` | 上电后的最短稳定等待 |
| `encoder_raw_stable_duration_sec` | `0.1` | 原始位置稳定持续时间 |
| `encoder_raw_stable_delta_counts` | `128` | 判定稳定的最大计数变化 |

关节数量虽然可以配置，但若改变 `num_joints`，还必须同步修改：

- `controllers.yaml` 中所有 joints 列表；
- `joint_geometry.yaml`；
- `calibrate.json`；
- 上层 API 的向量长度与关节映射。

## 13. 错误处理建议

整机 API 建议定义以下错误：

```text
NOT_READY
STALE_STATE
INVALID_JOINT
INVALID_VECTOR_LENGTH
INVALID_MODE
CONTROLLER_NOT_READY
CONTROLLER_SWITCH_FAILED
DRIVE_FAULT
ENABLE_TIMEOUT
MODE_TIMEOUT
ENCODER_RESTORE_NOT_READY
POSITION_OUT_OF_RANGE
VELOCITY_OUT_OF_RANGE
COMMAND_TIMEOUT
EMERGENCY_STOPPED
```

建议所有控制函数：

- 设置明确超时；
- 返回逐关节结果；
- 保存请求值与实际应用值；
- 检查消息时间戳和状态新鲜度；
- 不把“ROS publish 成功”当作“驱动执行成功”；
- 通过 feedback、status word、mode display 和 controller state 共同确认结果。

## 14. 上层 API 的最小实现清单

第一阶段建议至少实现：

```text
get_system_state
get_joint_state
get_controller_states
enable
disable
disable_all
reset_fault
set_mode
move_pp
start_csp
update_csp
stop_csp
set_csv_velocity
set_pv_velocity
stop_velocity
is_ready
has_fault
```

第二阶段可选：

```text
go_default
quick_stop
load_calibration
get_limits
maintenance_calibrate_linear
maintenance_calibrate_rotary_zero
maintenance_calibrate_rotary_range
reset_encoder_restore
```

普通业务 API 与维护 API 应分开授权。
