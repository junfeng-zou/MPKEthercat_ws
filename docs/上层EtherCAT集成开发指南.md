# MPKEthercat_ws — 上层对接技术说明

本文档描述本仓库中与**应用层 ROS 2 节点**直接相关的接口、配置契约及实现细节，便于与 `ethercat_driver_ros2` 子模块及 `mpk_ethercat_test` 包对齐联调。

---

## 1. 仓库与包边界

| 路径 | 内容 |
|------|------|
| `src/ethercat_driver_ros2/` | 上游 EtherCAT + ros2_control 硬件栈：`ethercat_driver`、`ethercat_interface`、`ethercat_generic_plugins`（含 `EcCiA402Drive`）、`ethercat_manager`、`ethercat_msgs` |
| `src/mpk_ethercat_test/` | MPK 侧：URDF、`motor_config.yaml`、`config/controller.yaml`、`MotorDataBroadcaster` 插件、测试 launch、GUI 脚本、`cyclic_mode_runner`、自定义消息 |
| `src/ethercat.json` | CiA402 相关对象字典条目参考（集成 PDO/SDO 时对照用） |

应用包若仅订阅/发命令：依赖 **`mpk_ethercat_test`**（消息类型）与 **`std_msgs`** 即可；SDO 服务需 **`ethercat_msgs`**。

---

## 2. 构建与运行前置

- ROS 2 **Humble**，`colcon` 工作空间，`source install/setup.bash`。
- IgH EtherCAT Master：本工程 CMake 侧常见假设为 **`/usr/local/etherlab`**（`ethercat_manager` 等与 EtherLab 链接）。

---

## 3. `ros2_control` 运行时拓扑

### 3.1 进程与参数

- **`controller_manager` / `ros2_control_node`**：加载 URDF 中 `<ros2_control>` 与 `controller.yaml`。
- **`controller_manager.update_rate`**（`config/controller.yaml`）：当前为 **1000 Hz**，与硬件侧 `control_frequency` 一致。

### 3.2 硬件插件（URDF）

- **插件**：`ethercat_driver/EthercatDriver`（`ethercat_driver` 包）。
- **硬件参数**（`multi_motor.urdf` / `single_motor.urdf`）：
  - `master_id`：整型，与 SDO 服务请求中的 `master_id` 及 `EcMasterAsync` 打开的主站一致（当前示例为 `0`）。
  - `control_frequency`：Hz，传入 `EthercatDriver`（见 `ethercat_driver.cpp` 中 `setCtrlFrequency`）；示例为 **1000**。

### 3.3 从站模块（每关节）

- **插件**：`ethercat_generic_plugins/EcCiA402Drive`。
- **参数**：`alias`、`position`（EtherCAT 从站在环上的 **position**，非逻辑轴号）、`slave_config`（YAML 路径）。

**路径差异**：

- `urdf/multi_motor.urdf`：`slave_config` 使用 `$(find mpk_ethercat_test)/config/motor_config.yaml`，由 `test_multi_motor.launch.py` 读入后把 `$(find …)` 替换为包路径。
- `urdf/single_motor.urdf`：`slave_config` 为**硬编码绝对路径**（当前指向本机 `.../src/mpk_ethercat_test/config/motor_config.yaml`），换机或改布局需改 URDF 或改 launch 注入方式。

---

## 4. `motor_config.yaml` — PDO / 启动 SDO 契约

文件：`src/mpk_ethercat_test/config/motor_config.yaml`。

| 项 | 值/说明 |
|----|---------|
| `vendor_id` / `product_id` | 从站识别 |
| `assign_activate` | DC/同步相关 |
| `auto_state_transitions` / `auto_fault_reset` | 当前为 `false`：CiA402 状态机由上层/应用通过 **0x6040** 推进 |
| **`sdo:`** | 启动时由 `EthercatDriver::configNetwork` 对每个模块执行 download（失败仅打 log，不中止加载的逻辑需结合驱动实现理解） |

**启动 SDO 示例（节选）**：

| Index | Sub | 类型 | 工程内含义 |
|-------|-----|------|------------|
| 0x60C2 | 1 | int8 | 插补时间数值（示例 `1`） |
| 0x60C2 | 2 | int8 | 时间基（示例 `-3` → 10⁻³ s），与 1 kHz 周期一致 |
| 0x6081 | 0 | uint32 | Profile Velocity（PP 等） |
| 0x6083 / 0x6084 | 0 | uint32 | Profile 加减速 |

**RPDO（写从站 / command_interface）** — `0x1600`：

| 对象 | 类型 | `command_interface` | `default` |
|------|------|---------------------|-----------|
| 0x6040 | uint16 | `control_word` | 0 |
| 0x607A | int32 | `position` | 0 |
| 0x60FF | int32 | `velocity` | 0 |
| 0x6060 | int8 | `op_mode` | **1**（注释：避免上电持续写无效模式 0） |

**TPDO（读从站 / state_interface）** — `0x1A00`：

| 对象 | 类型 | `state_interface` |
|------|------|-------------------|
| 0x6041 | uint16 | `status_word` |
| 0x6064 | int32 | `position` |
| 0x606C | int32 | `velocity` |
| 0x6061 | int8 | `op_mode` |

同一 YAML 可被多关节 `ec_module` 复用；**轴与从站**由 URDF 中各 `ec_module` 的 **`position`** 区分。

---

## 5. URDF 关节接口顺序（硬契约）

每个 `joint` 下 **`state_interface` 声明顺序**须与 `MotorDataBroadcaster::state_interface_configuration()` 中 `push_back` 顺序一致（见 `motor_data_broadcaster.cpp`）：

1. `position` → 2. `velocity` → 3. `status_word` → 4. `op_mode`

**`command_interface` 名称**须包含：`control_word`、`op_mode`、`position`、`velocity`（与上表 RPDO 映射一致）。顺序由 `ForwardCommandController` 的 `interface_name` 决定，与 `state_interface` 顺序无关。

---

## 6. `MotorDataBroadcaster`

### 6.1 标识

- **插件类**：`mpk_ethercat_test/MotorDataBroadcaster` → `MPKEthercat_test::MotorDataBroadcaster`（`motor_data_broadcaster_plugin.xml`）。
- **基类**：`controller_interface::ControllerInterface`。
- **`command_interface_configuration`**：`NONE`（不写命令）。
- **`state_interface_configuration`**：`INDIVIDUAL`，按电机循环绑定 §5 中四个接口名。

### 6.2 发布

- **相对话题**：`~/motor_data_topic`。
- **默认绝对话题**（控制器节点名为 `MotorDataBroadcaster`、无额外 namespace 时）：`/MotorDataBroadcaster/motor_data_topic`。
- **消息类型**：**仅** `mpk_ethercat_test/msg/MotorDataArray`（无单轴 `MotorData` 话题）。
- **QoS**：`rclcpp::SystemDefaultsQoS()`。
- **时间戳**：`msg.header.stamp` 与各 `motors[i].header.stamp` 使用控制器 `update()` 的 `time` 参数（controller_manager 时钟）。

### 6.3 `MotorDataArray` / `MotorData` 字段

`MotorDataArray.msg`：`std_msgs/Header header`，`MotorData[] motors`。

`MotorData.msg`：

```
std_msgs/Header header
string motor_name
int32 motor_index
uint16 status_word
int32 actual_position
int32 actual_velocity
int8 operation_mode
float64 actual_physical_position
float64 actual_physical_velocity
```

**语义与 PDO 对应**：

- `status_word` ← 0x6041  
- `actual_position` ← 0x6064（int32 计数）  
- `actual_velocity` ← 0x606C（int32，驱动报告；下游物理换算见下）  
- `operation_mode` ← 0x6061  

`motor_name`：来自参数 `motor_names[i]`。  
`motor_index`：实现里写死为轴索引 **`i`（0-based）**，与 `motor_names` 数组下标一致。

### 6.4 物理量换算（实现公式）

源码：`motor_data_broadcaster.cpp`（`update()`）。

- 记 `bias = position_bias_count`，`counts = encoder_counts_per_rev`，`lead = screw_lead_mm_per_rev`。
- **`actual_physical_position`（mm）**  
  `(bias - actual_position) * lead / counts`
- **`actual_physical_velocity`（mm/s）**  
  `-actual_velocity * lead / 1000.0`  
  注释说明：0x606C 按 **mrev/s** 处理；符号与位置方向约定一致（编码器速度正方向对应物理负方向）。

### 6.5 无效样本策略

对单轴，若 `position/velocity/status_word/op_mode` 任一对应 state 值为非有限（`!isfinite`）：

- 若该轴已有历史有效样本：本周期输出**上一有效样本**；`RCLCPP_WARN_THROTTLE` **2 s** 打一次 `"Invalid sample, using last valid value."`。
- 若无历史有效样本：该轴字段置 **0**；节流警告 `"No valid sample yet, publishing zero."`。

### 6.6 参数

**声明与加载**：

- `on_init()`：若节点上尚无 `motor_count` / `motor_names`，则声明默认值 **`motor_count=6`** 及 6 个默认 `motor_joint_1`…`motor_joint_6`。
- `on_configure()`：调用 `load_motor_configs()`，从参数服务器读取最终 `motor_count`、`motor_names` 及每轴 `motor_N.*`。

**`motor_names.size()` 与 `motor_count` 不一致**：取 **`min`**，并 `RCLCPP_WARN`。

**每轴参数前缀**：`motor_1`、`motor_2`、…（**1-based**，与轴序号一致）。

| 参数键 | 类型 | 默认（仅当 `declare_parameter` 触发时） | 说明 |
|--------|------|------------------------------------------|------|
| `motor_N.position_bias_count` | int | 0 | 见 §6.4 |
| `motor_N.encoder_counts_per_rev` | int | 131072 | 须 > 0 |
| `motor_N.screw_lead_mm_per_rev` | double | 10.0 | 须有限且非 0 |

**`add_on_set_parameters_callback`**（`on_set_parameters`）：

- 仅解析形如 **`motor_<索引>.<子项>`** 的参数名；**不**在回调里处理 `motor_count` / `motor_names` 变更。
- 校验失败时 `SetParametersResult.successful = false`：`encoder_counts_per_rev`≤0、`screw_lead_mm_per_rev` 非有限或为 0。

**推论**：修改 `motor_count` / `motor_names` 需重新 **configure** 控制器（或重启加载链）才能与 state 接口列表一致；运行时热改仅 `motor_N.*` 标定量是安全的（在实现语义内）。

### 6.7 `motor_count` 与 state 接口行数

`state_interface_configuration()` 按当前 `motor_count_` 申请 **4×motor_count** 个接口；与 URDF 关节数、`ForwardCommandController` 的 `joints` 列表必须一致，否则 hardware 或 CM 报错。

---

## 7. `ForwardCommandController` 四实例

定义：`src/mpk_ethercat_test/config/controller.yaml`。

| 控制器名 | `type` | `interface_name` | 绑定 command |
|----------|--------|------------------|--------------|
| `control_word_controller` | `forward_command_controller/ForwardCommandController` | `control_word` | 0x6040 |
| `op_mode_controller` | 同上 | `op_mode` | 0x6060 |
| `position_controller` | 同上 | `position` | 0x607A |
| `velocity_controller` | 同上 | `velocity` | 0x60FF |

**`joints`**：须与 URDF 中 joint 名列表一致；当前示例为三轴 `motor_joint_1`…`motor_joint_3`。

**订阅话题**（默认全局名，无 remap 时）：

- `/control_word_controller/commands`
- `/op_mode_controller/commands`
- `/position_controller/commands`
- `/velocity_controller/commands`

**消息类型**：`std_msgs/msg/Float64MultiArray`。  
**语义**：`data[i]` 对应 `joints[i]`；多轴长度须与 `joints` 长度一致。控制字、模式、位置、速度均以 **double 传递**，硬件侧再落到 uint16/int8/int32。

---

## 8. 测试 Launch 与控制器启动顺序

- **`launch/test_motor.launch.py`**：`single_motor.urdf` + 同一套 `controller.yaml`（注意 URDF 中 `slave_config` 绝对路径问题）。
- **`launch/test_multi_motor.launch.py`**：`multi_motor.urdf`（launch 内替换 `$(find …)`）+ `controller.yaml`；另起 `multi_motor_gui` 节点（参数与 CLI 与三轴一致）。

**Spawner 链**（两 launch 相同逻辑）：先 **`MotorDataBroadcaster`**；在其进程退出后依次 spawn **`control_word_controller`** → **`op_mode_controller`** → **`position_controller`** → **`velocity_controller`**（`OnProcessExit` 串联）。  
**注意**：`spawner` 正常执行完会退出，从而触发下一控制器；若 spawner 行为或 CM 版本导致未触发，需对照现场日志排查。

**未包含**：`ethercat_sdo_srv_server`（SDO 服务需单独启动，见 §10）。

---

## 9. 仓库内可执行/脚本与接口一致性

### 9.1 `cyclic_mode_runner`（`mpk_ethercat_test`）

- **默认订阅话题**：`/MotorDataBroadcaster/motor_data_topic`  
- **订阅消息类型**：`mpk_ethercat_test/msg/MotorData`  

当前 **`MotorDataBroadcaster` 仅发布 `MotorDataArray`**，二者**类型不匹配**；该二进制在未改源码或 remap 的情况下**不能**与现版广播器直接对接。若保留此工具，需改为订阅 `MotorDataArray` 并选取 `motors[k]`，或拆话题。

- **发布**：`std_msgs/Float64MultiArray` 至默认  
  `/control_word_controller/commands`、`/op_mode_controller/commands`、`/position_controller/commands`、`/velocity_controller/commands`。  
- **模式**：`--mode csp` → 内部写 **0x6060=8**；`csv` → **9**。周期发位置或速度指令；可选 `--csp-dynamic-target-topic` 订阅 `Float64MultiArray` 取动态目标。  
- **QoS**：订阅 `MotorData` 使用 `KeepLast(1)` + **BEST_EFFORT**。  
- **其它 CLI**：`--rate-hz`、`--target`、`--csp-start`、`--csp-step-per-cycle`、`--csp-speed`、`--require-enabled`、`--require-mode`、`--auto-enable`、各话题覆盖项等（见 `cyclic_mode_runner.cpp`）。

### 9.2 `multi_motor_gui.py`

- 订阅 **`MotorDataArray`**，默认 `/MotorDataBroadcaster/motor_data_topic`，QoS **BEST_EFFORT**，depth 10。  
- 向 §7 四个 `.../commands` 话题发布 **`Float64MultiArray`**；另可发 **`/csp_target_runtime/commands`**（供 CSP 动态目标；**无对应控制器**，仅为话题占位，需有消费者如改过的 `cyclic_mode_runner` 等）。  
- 多轴 **0x6040 状态机**自动推进逻辑按 `motors[i]` / `motor_index` 分支实现（详见脚本内 `MultiMotorDataSubscriber.callback`）。

### 9.3 `motor_data_gui.py`

- 订阅 **`MotorData`**，默认话题名与广播器相同。  
- 与当前 **`MotorDataArray` 专用广播器**不兼容；仅当存在其它发布 `MotorData` 的节点时可使用。

---

## 10. SDO 服务（`ethercat_manager`）

**可执行文件**：`ros2 run ethercat_manager ethercat_sdo_srv_server`（安装到 `lib/ethercat_manager`）。

**服务名**（节点默认名 `ethercat_sdo_srv_server`，无 namespace）：

| 服务 | 类型 |
|------|------|
| `/ethercat_manager/get_sdo` | `ethercat_msgs/srv/GetSdo` |
| `/ethercat_manager/set_sdo` | `ethercat_msgs/srv/SetSdo` |

**GetSdo 请求**：`int16 master_id`，`uint16 slave_position`，`uint16 sdo_index`，`uint8 sdo_subindex`，`string sdo_data_type`。  
**响应**：`bool success`，`string sdo_return_message`，`string sdo_return_value_string`，`float64 sdo_return_value`。

**SetSdo 请求**：`int16 master_id`，`int16 slave_position`，`int16 sdo_index`，`int16 sdo_subindex`，`string sdo_data_type`，`string sdo_value`。  
**响应**：`bool success`，`string sdo_return_message`。

**`sdo_data_type` 合法字符串**（与 `get_data_type` 严格相等匹配，见 `ethercat_manager/data_convertion_tools.hpp`）：  
`bool`, `int8`, `int16`, `int32`, `uint8`, `uint16`, `uint32`, `float`, `string`, `octet_string`, `unicode_string`, `int24`, `double`, `int40`, `int48`, `int56`, `int64`, `uint24`, `uint40`, `uint48`, `uint56`, `uint64`, `sm8`, `sm16`, `sm32`, `sm64`, `raw`。

**与实时栈关系**：SDO 走 `EcMasterAsync` 独立 open/close，与 `EthercatDriver` 周期主站并存时需注意主站/从站占用与文档中“非确定性”描述；`master_id` 须与硬件配置一致。

---

## 11. 上游 Sphinx（子模块内）

`src/ethercat_driver_ros2/ethercat_driver_ros2/sphinx/user_guide/` 含通用从站配置、SDO 异步等说明（如 `sdo_async_com.rst`）；本仓库 MPK 专用契约以 **本文 + `motor_config.yaml` + URDF + `controller.yaml`** 为准。

---

## 12. 对接清单（工程验收用）

1. URDF：`state_interface` 顺序 = §5；`command_interface` 四类齐全；`ec_module` `position` 与柜内接线一致。  
2. `controller.yaml`：`motor_count` / `motor_names` / 各 `motor_N.*` 与 URDF 关节数及命名一致；四个 `ForwardCommandController` 的 `joints` 与 URDF 一致。  
3. `controller_manager.update_rate` 与 URDF `control_frequency` 及 CiA402 0x60C2 配置一致（当前工程指向 1 ms）。  
4. 应用侧订阅 **`MotorDataArray`** 于 `/MotorDataBroadcaster/motor_data_topic`（或实际 remap 名）；勿假设存在 `MotorData` 话题。  
5. 命令侧四个 `Float64MultiArray` 向量维数等于 `joints` 长度。  
6. 若使用 SDO：单独启动 `ethercat_sdo_srv_server`，`master_id` / `slave_position` 与现场一致。  
7. `single_motor.urdf` 部署时检查 `slave_config` 路径是否仍有效。

---

*文档覆盖范围：截至本仓库当前 `mpk_ethercat_test` 与 `MotorDataBroadcaster` 实现；`cyclic_mode_runner` / `motor_data_gui.py` 与广播器消息类型不一致处已在 §9 标明。*
