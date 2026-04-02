#!/usr/bin/env python3
"""
GUI 界面：订阅 motor_data_topic，显示 EtherCAT 从站的
status_word（含含义）、position、velocity、op_mode（含含义）。
参考：0x6041 Status Word, 0x6061 Modes of operation display (Summit/CANopen)
"""

import sys
import argparse
import os
import subprocess
import signal
import re
from typing import Tuple, Optional, Callable

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from mpk_ethercat_test.msg import MotorData
    from std_msgs.msg import Float64MultiArray
except ImportError as e:
    print("请先 source 工作空间: source install/setup.bash", file=sys.stderr)
    raise e

try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    get_package_prefix = None

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError:
    print("需要 tkinter（一般随 Python 安装）", file=sys.stderr)
    sys.exit(1)


# ---------- Status Word 0x6041 解析 ----------
# 参考: https://drives.novantamotion.com/summit/0x6041-status-word
# Bit9=Remote, Bit10=Target reached(部分模式), Bit11=Internal limit active,
# Bit12/Bit13=Operation mode specific（随 op_mode 含义不同）

def _state_machine_str(sw: int) -> str:
    """状态机 bits 0-3, 5, 6 → 驱动状态。"""
    # 按截图里的判定表逐条匹配（x 表示该位不关心）
    #
    # - Ready/Switched on/Operation enabled/Quick stop active 需要区分 bit5，因此用 mask=0x6F (bits0-3,5,6)
    # - Not ready / Fault reaction active / Fault 这几行在表里 bit5 是 x，不应参与匹配，因此用 mask=0x4F (bits0-3,6)
    v_6f = sw & 0x6F
    v_4f = sw & 0x4F

    if v_4f == 0x00:
        return "Not ready to switch on"
    if v_4f == 0x40:
        return "Switch on disabled"
    if v_6f == 0x21:
        return "Ready to switch on"
    if v_6f == 0x23:
        return "Switched on"
    if v_6f == 0x27:
        return "Operation enabled"
    if v_6f == 0x07:
        return "Quick stop active"
    if v_4f == 0x0F:
        return "Fault reaction active"
    if v_4f == 0x08:
        return "Fault"

    return f"Unknown state (sw=0x{sw & 0xFFFF:04X}, sw&0x6F=0x{v_6f:02X}, sw&0x4F=0x{v_4f:02X})"


def _mode_specific_bits_str(sw: int, op_mode: int) -> str:
    """只关注 Bit10 / Bit12 / Bit13（以及必要的 Bit14），不显示 Bit9 和 Bit11。"""
    bit10 = (sw >> 10) & 1  # Target reached（部分模式下有效）
    bit12 = (sw >> 12) & 1  # Operation mode specific
    bit13 = (sw >> 13) & 1  # Operation mode specific


    parts = []

    # Bit10 = Target reached 在部分模式下有效；Bit12/Bit13/Bit14 = 模式相关含义
    if op_mode == 6:
        # Homing: Bit13=Homing error, Bit12=Homing attained, Bit10=Target reached
        # 根据 bit13/bit12/bit10 组合解码含义
        homing_key = (bit13, bit12, bit10)
        homing_map = {
            (0, 0, 0): "Homing procedure is in progress",
            (0, 0, 1): "Homing procedure is interrupted or not started",
            (0, 1, 0): "Homing is attained but target is not reached",
            (0, 1, 1): "Homing mode carried out successfully",
            (1, 0, 0): "Homing error occurred; Homing mode carried out not successfully; Velocity is not zero",
            (1, 0, 1): "Homing error occurred; Homing mode carried out not successfully; Velocity is zero",
            (1, 1, 0): "Reserved",
            (1, 1, 1): "Reserved",
        }
        desc = homing_map.get(homing_key, "Unknown homing state")
        parts.append(desc)
    elif op_mode in (8, 9, 10):
        # Cyclic sync (CSP/CSV/CST): Bit13=Following error, Bit12=Drive follows the command value
        parts.append(f"Following error: {'Following error' if bit13 else 'No following error'}")
        parts.append(f"Drive follows command: {'The drive follows the target value' if bit12 else 'The drive does not follow the target value'}")
    elif op_mode == 3:
        # Profile velocity: Bit13=Following error, Bit12=Speed, Bit10=Target reached
        parts.append(f"Target reached: {'Target velocity reached' if bit10 else 'Target velocity not reached'}")
        parts.append(f"Speed: {'Speed is zero' if bit12 else 'Speed is not zero'}")
        parts.append(f"Following error: {'Following error' if bit13 else 'No following error'}")
    elif op_mode == 1:
        # Profile position: Bit13=Following error, Bit12=Set-point ack, Bit10=Target reached
        parts.append(f"Target reached: {'Target position reached' if bit10 else 'Target position not reached'}")
        parts.append(f"Set-point acknowledge: {'Trajectory generator has assumed the positioning values' if bit12 else 'Trajectory generator has not assumed the positioning values'}")
        parts.append(f"Following error: {'Following error' if bit13 else 'No following error'}")
    else:
        parts.append(f"Bit10 Target reached: {'Yes' if bit10 else 'No'}")

    return " | ".join(parts)


def decode_status_word(sw: int, op_mode: int) -> Tuple[str, str]:
    """
    将 status_word 编码解析为驱动状态与模式相关位说明。
    返回 (状态机描述, Bit10/12/13/14 描述)。
    """
    return _state_machine_str(sw), _mode_specific_bits_str(sw, op_mode)


# ---------- Operation Mode 0x6061 解析 ----------
# 参考: https://drives.novantamotion.com/summit/0x6061-operation-mode-display
def decode_operation_mode(op: int) -> str:
    """将 operation_mode 编码解析为运行模式描述。"""
    mode_map = {
        -4: "Current amplifier",
        -3: "Cyclic sync current mode",
        -2: "Current mode",
        -1: "Voltage mode",
         1: "Profile position mode",
         3: "Profile velocity mode",
         6: "Homing mode",
         8: "Cyclic sync position mode",
         9: "Cyclic sync velocity mode",
        10: "Cyclic sync torque mode",
    }
    return mode_map.get(op, f"Reserved/Unknown ({op})")


# CiA 402 状态机自动转换：到 Switched on 之前自动发控制字，用户只操作 Switched on -> Operation enable
CONTROLWORD_SHUTDOWN = 0x0006   # Switch on disabled -> Ready to switch on
CONTROLWORD_SWITCH_ON = 0x0007  # Ready to switch on -> Switched on


class MotorDataSubscriber(Node):
    """订阅 MotorData，更新 GUI；若提供 send_control_word_fn 则自动执行到 Switched on 的状态转换。"""

    def __init__(self, topic: str, update_gui_cb, send_control_word_fn: Optional[Callable[[int], None]] = None):
        super().__init__("motor_data_gui_subscriber")
        self.update_gui_cb = update_gui_cb
        self._send_cw = send_control_word_fn
        self._last_state: Optional[str] = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            MotorData,
            topic,
            self.callback,
            qos,
        )
        self.get_logger().info(f"Subscribed to: {topic}")
        if self._send_cw is not None:
            self.get_logger().info("Auto state transition enabled: Switch on disabled -> 0x0006, Ready to switch on -> 0x0007")

    def callback(self, msg: MotorData):
        state_str = _state_machine_str(msg.status_word)
        if self._send_cw is not None:
            if state_str == "Switch on disabled" and self._last_state != "Switch on disabled":
                self._send_cw(CONTROLWORD_SHUTDOWN)
            elif state_str == "Ready to switch on" and self._last_state != "Ready to switch on":
                self._send_cw(CONTROLWORD_SWITCH_ON)
            self._last_state = state_str
        self.update_gui_cb(
            status_word=msg.status_word,
            position=msg.actual_position,
            velocity=msg.actual_velocity,
            operation_mode=msg.operation_mode,
            physical_position=msg.actual_physical_position,
            physical_velocity=msg.actual_physical_velocity,
        )


class MotorCommandPublisher(Node):
    """发布控制命令到各 ForwardCommandController 的 /commands 话题。"""

    def __init__(
        self,
        control_word_topic: str = "/control_word_controller/commands",
        op_mode_topic: str = "/op_mode_controller/commands",
        position_topic: str = "/position_controller/commands",
        velocity_topic: str = "/velocity_controller/commands",
        csp_dynamic_target_topic: str = "/csp_target_runtime/commands",
    ):
        super().__init__("motor_command_gui_publisher")
        self.pub_control_word = self.create_publisher(Float64MultiArray, control_word_topic, 10)
        self.pub_op_mode = self.create_publisher(Float64MultiArray, op_mode_topic, 10)
        self.pub_position = self.create_publisher(Float64MultiArray, position_topic, 10)
        self.pub_velocity = self.create_publisher(Float64MultiArray, velocity_topic, 10)
        self.pub_csp_dynamic_target = self.create_publisher(Float64MultiArray, csp_dynamic_target_topic, 10)
        self._last_control_word: Optional[int] = None

        self.get_logger().info(f"Control topics:")
        self.get_logger().info(f"  control_word: {control_word_topic}")
        self.get_logger().info(f"  op_mode:      {op_mode_topic}")
        self.get_logger().info(f"  position:     {position_topic}")
        self.get_logger().info(f"  velocity:     {velocity_topic}")
        self.get_logger().info(f"  csp_target:   {csp_dynamic_target_topic}")

    @staticmethod
    def _msg1(v: float) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [float(v)]
        return msg

    def send_control_word(self, cw: int):
        cw16 = int(cw) & 0xFFFF
        self._last_control_word = cw16
        self.pub_control_word.publish(self._msg1(float(cw16)))

    def get_last_control_word(self) -> Optional[int]:
        return self._last_control_word

    def send_op_mode(self, mode: int):
        self.pub_op_mode.publish(self._msg1(float(int(mode))))

    def send_position(self, pos: int):
        self.pub_position.publish(self._msg1(float(int(pos))))

    def send_velocity(self, vel: int):
        self.pub_velocity.publish(self._msg1(float(int(vel))))

    def send_csp_dynamic_target(self, target_pos: int):
        self.pub_csp_dynamic_target.publish(self._msg1(float(int(target_pos))))


def main():
    parser = argparse.ArgumentParser(description="Motor Data GUI - 显示 EtherCAT 从站 status_word/position/velocity/op_mode")
    parser.add_argument(
        "--topic",
        default="/MotorDataBroadcaster/motor_data_topic",
        help="MotorData 话题名（默认: motor_data_topic）",
    )
    parser.add_argument("--control-word-topic", default="/control_word_controller/commands")
    parser.add_argument("--op-mode-topic", default="/op_mode_controller/commands")
    parser.add_argument("--position-topic", default="/position_controller/commands")
    parser.add_argument("--velocity-topic", default="/velocity_controller/commands")
    parser.add_argument("--csp-dynamic-target-topic", default="/csp_target_runtime/commands")
    args = parser.parse_args()

    rclpy.init()
    root = tk.Tk()
    root.title("Motor Data - EtherCAT 从站状态")
    root.geometry("860x520")
    root.resizable(True, True)

    # 显示用的变量（在主线程更新）
    status_text = tk.StringVar(value="—")
    status_bits_text = tk.StringVar(value="—")  # Bit10/12/13 等按 op_mode 的含义
    position_text = tk.StringVar(value="—")
    physical_position_text = tk.StringVar(value="—")
    velocity_text = tk.StringVar(value="—")
    physical_velocity_text = tk.StringVar(value="—")
    op_mode_text = tk.StringVar(value="—")
    raw_status = tk.StringVar(value="0x0000")
    raw_op_mode = tk.StringVar(value="—")
    status_send = tk.StringVar(value="—")
    latest_status_word: Optional[int] = None
    latest_operation_mode: Optional[int] = None
    latest_position: Optional[float] = None
    latest_velocity: Optional[float] = None
    latest_physical_position: Optional[float] = None
    active_bias_count: Optional[int] = None
    encoder_counts_per_rev = 131072.0
    screw_lead_mm_per_rev = 10.0
    cyclic_running = False
    cyclic_after_id = None
    cyclic_guard_msg = ""
    cyclic_last_sent_pos: Optional[float] = None
    cyclic_last_sent_vel: Optional[float] = None
    current_cyclic_mode_code: Optional[int] = None
    cyclic_runner_proc: Optional[subprocess.Popen] = None
    calib_running = False
    calib_after_id = None
    calib_stage = "idle"
    calib_base_pos: Optional[int] = None
    calib_last_cmd_pos: Optional[int] = None
    calib_rate_hz_var = tk.StringVar(value="500")
    calib_speed_var = tk.StringVar(value="200000")
    position_bias_text = tk.StringVar(value="—")
    calib_sent_position_text = tk.StringVar(value="—")

    def _apply_runtime_bias(new_bias: int) -> bool:
        """将最新标定 bias 立即写入运行中的 MotorDataBroadcaster 参数。"""
        candidate_nodes = [
            "/MotorDataBroadcaster",
            "/controller_manager/MotorDataBroadcaster",
        ]
        for node_name in candidate_nodes:
            try:
                r = subprocess.run(
                    ["ros2", "param", "set", node_name, "position_bias_count", str(int(new_bias))],
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                    check=False,
                )
                if r.returncode == 0:
                    return True
            except Exception:
                pass
        return False

    def _persist_bias_to_controller_yaml(new_bias: int) -> bool:
        """将标定得到的 bias 写回 controller.yaml，供下次启动默认生效。"""
        this_file = os.path.realpath(__file__)
        ws_root = os.path.realpath(os.path.join(os.path.dirname(this_file), "..", "..", "..", ".."))
        candidate_paths = []
        if get_package_prefix is not None:
            try:
                pkg_prefix = os.path.realpath(get_package_prefix("mpk_ethercat_test"))
                # install 路径（source install/setup.bash 后可用）
                candidate_paths.append(
                    os.path.join(pkg_prefix, "share", "mpk_ethercat_test", "config", "controller.yaml")
                )
                # 推导工作空间根目录，优先写回 src 配置文件
                if os.path.basename(os.path.dirname(pkg_prefix)) == "install":
                    inferred_ws = os.path.realpath(os.path.join(pkg_prefix, "..", ".."))
                    candidate_paths.insert(
                        0, os.path.join(inferred_ws, "src", "mpk_ethercat_test", "config", "controller.yaml")
                    )
            except Exception:
                pass
        # 回退路径（兼容直接从源码运行）
        candidate_paths += [
            os.path.join(ws_root, "src", "mpk_ethercat_test", "config", "controller.yaml"),
            os.path.join(ws_root, "install", "mpk_ethercat_test", "share", "mpk_ethercat_test", "config", "controller.yaml"),
        ]
        target = ""
        for p in candidate_paths:
            if os.path.isfile(p):
                target = p
                break
        if not target:
            return False
        try:
            with open(target, "r", encoding="utf-8") as f:
                lines = f.readlines()

            in_broadcaster = False
            in_ros_params = False
            replaced = False
            out_lines = []
            for line in lines:
                stripped = line.strip()
                if re.match(r"^MotorDataBroadcaster:\s*$", stripped):
                    in_broadcaster = True
                    in_ros_params = False
                    out_lines.append(line)
                    continue
                if in_broadcaster and re.match(r"^ros__parameters:\s*$", stripped):
                    in_ros_params = True
                    out_lines.append(line)
                    continue
                if in_broadcaster and in_ros_params:
                    # 离开参数块
                    if stripped and (not line.startswith(" ") and not line.startswith("\t")):
                        in_broadcaster = False
                        in_ros_params = False
                    elif re.match(r"^\s*position_bias_count\s*:", line):
                        indent = re.match(r"^(\s*)", line).group(1)
                        out_lines.append(f"{indent}position_bias_count: {int(new_bias)}\n")
                        replaced = True
                        continue
                out_lines.append(line)

            if not replaced:
                return False

            with open(target, "w", encoding="utf-8") as f:
                f.writelines(out_lines)
            return True
        except Exception:
            return False

    def _physical_to_encoder_position(physical_pos: float) -> Optional[int]:
        """将物理量目标（当前关节1为 mm）转换为编码器位置命令。"""
        nonlocal active_bias_count
        if active_bias_count is None:
            return None
        counts_per_mm = encoder_counts_per_rev / screw_lead_mm_per_rev
        return int(round(float(active_bias_count) - float(physical_pos) * counts_per_mm))

    def _mmps_to_csp_countsps(v_mmps: float) -> float:
        """CSP限速使用位置编码器单位：counts/s（方向与物理相反）。"""
        counts_per_mm = encoder_counts_per_rev / screw_lead_mm_per_rev
        return -float(v_mmps) * counts_per_mm

    def _mmps2_to_csp_countsps2(v_mmps2: float) -> float:
        """CSP相关加速度量，使用 counts/s²。"""
        counts_per_mm = encoder_counts_per_rev / screw_lead_mm_per_rev
        return float(v_mmps2) * counts_per_mm

    def _mmps_to_csv_mrevs(v_mmps: float) -> float:
        """CSV(0x60FF)速度单位：mrev/s（方向与物理相反）。"""
        return -float(v_mmps) * 1000.0 / screw_lead_mm_per_rev

    def _mmps2_to_csv_mrevs2(v_mmps2: float) -> float:
        """CSV速度变化率对应单位：mrev/s²。"""
        return float(v_mmps2) * 1000.0 / screw_lead_mm_per_rev

    def _clamp_csp_physical(v: float) -> float:
        return max(0.0, min(274.7, float(v)))

    def update_gui(
        status_word: int,
        position: int,
        velocity: int,
        operation_mode: int,
        physical_position: float,
        physical_velocity: float,
    ):
        root.after(0, lambda: _do_update(
            status_word, position, velocity, operation_mode, physical_position, physical_velocity
        ))

    def _do_update(
        status_word: int,
        position: int,
        velocity: int,
        operation_mode: int,
        physical_position: float,
        physical_velocity: float,
    ):
        nonlocal latest_status_word, latest_operation_mode, latest_position, latest_velocity, latest_physical_position, active_bias_count
        nonlocal calib_running
        latest_status_word = int(status_word)
        latest_operation_mode = int(operation_mode)
        latest_position = float(position)
        latest_velocity = float(velocity)
        latest_physical_position = float(physical_position)
        # 由反馈的物理量反推当前生效的 bias，保证滑块下发使用最新转换参数
        counts_per_mm = encoder_counts_per_rev / screw_lead_mm_per_rev
        active_bias_count = int(round(float(position) + float(physical_position) * counts_per_mm))
        state_str, bits_str = decode_status_word(status_word, operation_mode)
        status_text.set(state_str)
        status_bits_text.set(bits_str)
        raw_status.set(f"0x{status_word & 0xFFFF:04X}")
        position_text.set(str(position))
        physical_position_text.set(f"{float(physical_position):.6f}")
        velocity_text.set(str(velocity))
        physical_velocity_text.set(f"{float(physical_velocity):.6f}")
        op_mode_text.set(decode_operation_mode(operation_mode))
        raw_op_mode.set(str(operation_mode))
        # 仅在明确异常时自动停止周期流；等待使能阶段不应被误判为异常
        if cyclic_running and state_str in (
            "Quick stop active",
            "Fault",
            "Fault reaction active",
        ):
            _cyclic_stop_internal(show_msg=False)
            status_send.set(f"周期下发自动停止：当前状态为 {state_str}")
        if calib_running and state_str == "Fault":
            position_bias_text.set(str(position))
            applied = _apply_runtime_bias(int(position))
            persisted = _persist_bias_to_controller_yaml(int(position))
            _calib_stop_internal(show_msg=False)
            if applied and persisted:
                status_send.set(
                    f"标定完成：bias={position}，已立即生效并写入 controller.yaml"
                )
            elif applied and not persisted:
                status_send.set(
                    f"标定完成：bias={position} 已立即生效，但写入 controller.yaml 失败"
                )
            elif (not applied) and persisted:
                status_send.set(
                    f"标定完成：bias={position} 已写入 controller.yaml，但运行时应用失败"
                )
            else:
                status_send.set(
                    f"标定完成：bias={position}；运行时应用和写入yaml都失败，请手动处理"
                )

    cmd_node = MotorCommandPublisher(
        control_word_topic=args.control_word_topic,
        op_mode_topic=args.op_mode_topic,
        position_topic=args.position_topic,
        velocity_topic=args.velocity_topic,
        csp_dynamic_target_topic=args.csp_dynamic_target_topic,
    )
    sub_node = MotorDataSubscriber(
        args.topic,
        update_gui,
        # 开启自动状态机转换：Switch on disabled -> 0x0006, Ready to switch on -> 0x0007
        send_control_word_fn=cmd_node.send_control_word,
    )

    # 布局
    pad = 12
    title_font = tkfont.Font(root, size=14, weight="bold")
    label_font = tkfont.Font(root, size=10)
    value_font = tkfont.Font(root, size=11)

    # 两列布局：左侧显示，右侧控制
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)

    left = tk.Frame(root)
    right = tk.Frame(root)
    left.grid(row=0, column=0, sticky="nsew")
    right.grid(row=0, column=1, sticky="nsew")
    left.grid_columnconfigure(1, weight=1)
    right.grid_columnconfigure(1, weight=1)

    row = 0
    tk.Label(left, text="Motor Data (EtherCAT)", font=title_font).grid(row=row, column=0, columnspan=4, pady=(pad, pad * 2), sticky="w", padx=pad)
    row += 1

    def add_row(name: str, value_var: tk.StringVar, extra_var: tk.StringVar = None, extra_label: str = ""):
        nonlocal row
        tk.Label(left, text=name + ":", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=4)
        tk.Label(left, textvariable=value_var, font=value_font, anchor="w", fg="#1a1a1a").grid(row=row, column=1, sticky="w", padx=pad, pady=4)
        if extra_var and extra_label:
            tk.Label(left, text=extra_label, font=label_font, anchor="w").grid(row=row, column=2, sticky="w", padx=(0, 4), pady=4)
            tk.Label(left, textvariable=extra_var, font=tkfont.Font(root, size=9), fg="gray", anchor="w").grid(row=row, column=3, sticky="w", padx=pad, pady=4)
        row += 1

    add_row("Status Word (状态机)", status_text, raw_status, "编码:")
    # 仅显示 Bit10/12/13/14（模式相关含义）
    tk.Label(left, text="Status Word (Bit10/12/13/14):", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=4)
    tk.Label(left, textvariable=status_bits_text, font=tkfont.Font(root, size=9), anchor="w", fg="#333", wraplength=340).grid(row=row, column=1, columnspan=3, sticky="w", padx=pad, pady=4)
    row += 1
    add_row("Position", position_text)
    add_row("Physical Position", physical_position_text)
    add_row("Velocity", velocity_text)
    add_row("Physical Velocity", physical_velocity_text)
    add_row("Op Mode (含义)", op_mode_text, raw_op_mode, "编码:")

    row += 1
    tk.Label(left, text=f"订阅: {args.topic}", font=tkfont.Font(root, size=9), fg="gray").grid(row=row, column=0, columnspan=4, sticky="w", padx=pad, pady=pad)

    # ----------------- 控制面板（右侧） -----------------
    rrow = 0
    tk.Label(right, text="控制 (写 command interfaces)", font=title_font).grid(row=rrow, column=0, columnspan=2, pady=(pad, pad), sticky="w", padx=pad)
    rrow += 1

    def _add_entry(label: str, default: str) -> tk.Entry:
        nonlocal rrow
        tk.Label(right, text=label, font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=6)
        e = tk.Entry(right, width=18)
        e.insert(0, default)
        e.grid(row=rrow, column=1, sticky="w", padx=pad, pady=6)
        rrow += 1
        return e

    pos_entry = _add_entry("目标位置 (PP, int32):", "0")
    vel_entry = _add_entry("目标速度 (PV/CSV, mm/s):", "0")

    tk.Label(right, text="CSP目标位置(物理量):", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=6)
    csp_target_var = tk.DoubleVar(value=0.0)
    csp_scale = tk.Scale(
        right,
        from_=0.0,
        to=274.7,
        orient=tk.HORIZONTAL,
        variable=csp_target_var,
        length=220,
        resolution=0.1,
    )
    csp_scale.grid(row=rrow, column=1, sticky="w", padx=pad, pady=6)
    rrow += 1

    tk.Label(right, text="CSP滑块值(物理量):", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=2)
    csp_target_text = tk.StringVar(value="0.0")
    tk.Label(right, textvariable=csp_target_text, font=tkfont.Font(root, size=9), anchor="w", fg="#333").grid(
        row=rrow, column=1, sticky="w", padx=pad, pady=2
    )
    rrow += 1

    def _on_csp_slider_changed(_=None):
        csp_target_text.set(f"{float(csp_target_var.get()):.3f}")
        mode_code = _cyclic_mode_code()
        if cyclic_running and mode_code == 8 and cyclic_runner_proc is not None:
            physical_target = _clamp_csp_physical(float(csp_target_var.get()))
            target_enc = _physical_to_encoder_position(physical_target)
            if target_enc is not None:
                cmd_node.send_csp_dynamic_target(int(target_enc))

    csp_scale.configure(command=_on_csp_slider_changed)

    # op_mode 允许输入或选择常用值
    tk.Label(right, text="运行模式 op_mode:", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=6)
    op_mode_var = tk.StringVar(value="1 (PP)")
    op_mode_menu = tk.OptionMenu(
        right,
        op_mode_var,
        "1 (PP)",
        "3 (PV)",
        "6 (Homing)",
        "8 (CSP)",
        "9 (CSV)",
    )
    op_mode_menu.grid(row=rrow, column=1, sticky="w", padx=pad, pady=6)
    rrow += 1

    tk.Label(right, textvariable=status_send, font=tkfont.Font(root, size=9), fg="gray", anchor="w", wraplength=340).grid(
        row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, pad)
    )
    rrow += 1

    def _parse_int(s: str) -> Optional[int]:
        s = s.strip()
        if not s:
            return None
        try:
            return int(s, 0)  # 支持 123 / 0x7B
        except ValueError:
            return None

    def _op_mode_value() -> Optional[int]:
        # 允许 "8 (CSP)" 这种格式
        t = op_mode_var.get().strip()
        if not t:
            return None
        if " " in t:
            t = t.split(" ", 1)[0]
        return _parse_int(t)

    def _refresh_position_input_mode(*_):
        """联动启停位置输入控件：PP 用输入框，CSP 用滑块，其它模式都禁用。"""
        mode_code = _op_mode_value()
        if mode_code == 1:  # PP
            pos_entry.configure(state="normal")
            csp_scale.configure(state="disabled")
        elif mode_code == 8:  # CSP
            pos_entry.configure(state="disabled")
            csp_scale.configure(state="normal")
        else:
            pos_entry.configure(state="disabled")
            csp_scale.configure(state="disabled")

    op_mode_var.trace_add("write", _refresh_position_input_mode)
    _refresh_position_input_mode()

    def send_op_mode():
        m = _op_mode_value()
        if m is None:
            status_send.set("op_mode 输入非法")
            return
        cmd_node.send_op_mode(m)
        status_send.set(f"已发送 op_mode={m} 到 {args.op_mode_topic}")

    def send_position():
        mode_code = _op_mode_value()
        if mode_code == 8:
            physical_target = _clamp_csp_physical(float(csp_target_var.get()))
            enc = _physical_to_encoder_position(physical_target)
            if enc is None:
                status_send.set("CSP发送失败：尚未获得有效bias，无法从物理量转换到编码器位置")
                return
            p = int(enc)
        else:
            p = _parse_int(pos_entry.get())
            if p is None:
                status_send.set("目标位置输入非法")
                return
        cmd_node.send_position(p)
        if mode_code == 8:
            status_send.set(
                f"已发送 CSP 物理目标={physical_target:.3f}，编码器position={p} 到 {args.position_topic}"
            )
        elif mode_code == 1:
            send_pp_start_pulse()
            status_send.set(f"已发送 PP position={p} 到 {args.position_topic}，并触发 Start 脉冲")
        else:
            status_send.set(f"已发送 position={p} 到 {args.position_topic}")

    def send_velocity():
        v_mmps = _parse_float(vel_entry.get())
        if v_mmps is None:
            status_send.set("目标速度输入非法")
            return
        v = int(round(_mmps_to_csv_mrevs(v_mmps)))
        cmd_node.send_velocity(v)
        status_send.set(f"已发送 物理速度={v_mmps:.3f} mm/s -> 编码器velocity={v} 到 {args.velocity_topic}")

    def send_control_word(cw: int, label: str):
        cmd_node.send_control_word(cw)
        status_send.set(f"已发送 control_word=0x{cw & 0xFFFF:04X} ({label}) 到 {args.control_word_topic}")

    def send_pp_start_pulse():
        # PP: 基于当前控制字，仅翻转 bit4 产生 0->1 上升沿，其它 bit 不变。
        current_cw = cmd_node.get_last_control_word()
        if current_cw is None:
            status_send.set("PP Start 失败：尚无当前控制字缓存，请先发送一次控制字")
            return

        cw_low = current_cw & (~0x0010)   # bit4 = 0
        cw_high = cw_low | 0x0010         # bit4 = 1（上升沿）

        cmd_node.send_control_word(cw_low)
        root.after(20, lambda: cmd_node.send_control_word(cw_high))
        root.after(100, lambda: cmd_node.send_control_word(cw_low))
        status_send.set(
            f"已发送 PP Start 脉冲(仅bit4翻转): 0x{cw_low:04X}->0x{cw_high:04X}->0x{cw_low:04X}"
        )

    def _parse_float(s: str) -> Optional[float]:
        s = s.strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            return None

    rate_hz_var = tk.StringVar(value="500")
    csp_speed_limit_var = tk.StringVar(value="20")
    csv_acc_limit_var = tk.StringVar(value="200")
    cyclic_sent_value_text = tk.StringVar(value="—")

    def _cyclic_stop_internal(show_msg: bool = True):
        nonlocal cyclic_running, cyclic_after_id, cyclic_guard_msg, cyclic_last_sent_pos, cyclic_last_sent_vel
        nonlocal cyclic_runner_proc, current_cyclic_mode_code
        was_csv = (current_cyclic_mode_code == 9)
        cyclic_running = False
        cyclic_guard_msg = ""
        cyclic_last_sent_pos = None
        cyclic_last_sent_vel = None
        if cyclic_runner_proc is not None:
            try:
                # 优先优雅中断，让 runner 走 KeyboardInterrupt + cleanup，减少 rcl context 异常
                cyclic_runner_proc.send_signal(signal.SIGINT)
                cyclic_runner_proc.wait(timeout=1.0)
            except Exception:
                try:
                    cyclic_runner_proc.terminate()
                    cyclic_runner_proc.wait(timeout=1.0)
                except Exception:
                    pass
            cyclic_runner_proc = None
        if cyclic_after_id is not None:
            root.after_cancel(cyclic_after_id)
            cyclic_after_id = None
        if was_csv:
            # CSV 停止时主动清零，避免控制器保持最后一个非零速度
            cmd_node.send_velocity(0)
        if show_msg:
            status_send.set("已停止周期下发")

    def _cyclic_mode_code() -> Optional[int]:
        # 周期模式与手动 op_mode 共用同一个选择，避免界面歧义
        t = op_mode_var.get().strip()
        if not t:
            return None
        if " " in t:
            t = t.split(" ", 1)[0]
        return _parse_int(t)

    def _align_targets_for_mode(mode_code: int):
        """模式切换时做安全对齐，避免旧模式目标造成突跳。"""
        if mode_code == 8:
            if latest_physical_position is not None and latest_position is not None:
                raw_phys = float(latest_physical_position)
                aligned_phys = _clamp_csp_physical(raw_phys)
                aligned_enc = _physical_to_encoder_position(aligned_phys)
                if aligned_enc is None:
                    aligned_enc = int(round(latest_position))
                csp_target_var.set(aligned_phys)
                csp_target_text.set(f"{aligned_phys:.3f}")
                cmd_node.send_position(aligned_enc)
                if abs(raw_phys - aligned_phys) > 1e-6:
                    cyclic_sent_value_text.set(
                        f"CSP对齐: 当前物理量 {raw_phys:.3f} 超出滑块范围，夹紧到 {aligned_phys:.3f} -> 编码器 {aligned_enc}"
                    )
                else:
                    cyclic_sent_value_text.set(
                        f"CSP对齐: 物理量 {aligned_phys:.3f} -> 编码器 {aligned_enc}"
                    )
        elif mode_code == 9:
            vel_entry.delete(0, tk.END)
            vel_entry.insert(0, "0")
            cmd_node.send_velocity(0)
            cyclic_sent_value_text.set("CSV切换保护: 速度置零")

    def _cyclic_tick():
        nonlocal cyclic_after_id, cyclic_guard_msg, cyclic_last_sent_pos, cyclic_last_sent_vel
        if not cyclic_running:
            return

        mode_code = _cyclic_mode_code()
        if mode_code not in (8, 9):
            _cyclic_stop_internal(show_msg=False)
            status_send.set("周期下发停止：模式必须为 8(CSP) 或 9(CSV)")
            return

        state_ok = latest_status_word is not None and _state_machine_str(latest_status_word) == "Operation enabled"
        mode_ok = latest_operation_mode is not None and int(latest_operation_mode) == int(mode_code)
        if not state_ok:
            msg = "周期下发等待：驱动未到 Operation enabled"
            if cyclic_guard_msg != msg:
                status_send.set(msg)
                cyclic_guard_msg = msg
        elif not mode_ok:
            msg = f"周期下发等待：0x6061={latest_operation_mode}，期望={mode_code}"
            if cyclic_guard_msg != msg:
                status_send.set(msg)
                cyclic_guard_msg = msg
        else:
            cyclic_guard_msg = ""
            hz = _parse_float(rate_hz_var.get())
            if hz is None or hz <= 0.0:
                _cyclic_stop_internal(show_msg=False)
                status_send.set("周期下发停止：频率输入非法")
                return
            dt = 1.0 / hz
            if mode_code == 8:
                physical_target = _clamp_csp_physical(float(csp_target_var.get()))
                target_enc = _physical_to_encoder_position(physical_target)
                if target_enc is None:
                    _cyclic_stop_internal(show_msg=False)
                    status_send.set("周期下发停止：尚未获得有效bias，无法从物理量转换到编码器位置")
                    return
                target_p = float(target_enc)
                limit = _parse_float(csp_speed_limit_var.get())
                if limit is None or limit <= 0.0:
                    _cyclic_stop_internal(show_msg=False)
                    status_send.set("周期下发停止：CSP 上限(mm/s)输入非法")
                    return
                limit_counts = _mmps_to_csp_countsps(limit)
                if cyclic_last_sent_pos is None:
                    if latest_position is not None:
                        cyclic_last_sent_pos = float(latest_position)
                    else:
                        cyclic_last_sent_pos = float(target_p)
                step_max = limit_counts * dt
                delta = float(target_p) - cyclic_last_sent_pos
                if delta > step_max:
                    cmd_p = cyclic_last_sent_pos + step_max
                elif delta < -step_max:
                    cmd_p = cyclic_last_sent_pos - step_max
                else:
                    cmd_p = float(target_p)
                cyclic_last_sent_pos = cmd_p
                cmd_node.send_position(int(round(cmd_p)))
                cyclic_sent_value_text.set(
                    f"CSP下发: 物理目标 {physical_target:.3f}, 编码器 {int(round(cmd_p))} (目标 {int(round(target_p))})"
                )
            else:
                target_v_mmps = _parse_float(vel_entry.get())
                limit = _parse_float(csv_acc_limit_var.get())
                if target_v_mmps is None:
                    _cyclic_stop_internal(show_msg=False)
                    status_send.set("周期下发停止：目标速度输入非法")
                    return
                target_v = _mmps_to_csv_mrevs(target_v_mmps)
                if limit is None or limit <= 0.0:
                    _cyclic_stop_internal(show_msg=False)
                    status_send.set("周期下发停止：CSV 上限(mm/s²)输入非法")
                    return
                limit_counts = _mmps2_to_csv_mrevs2(limit)
                if cyclic_last_sent_vel is None:
                    if latest_velocity is not None:
                        cyclic_last_sent_vel = float(latest_velocity)
                    else:
                        cyclic_last_sent_vel = 0.0
                step_max = limit_counts * dt
                delta = float(target_v) - cyclic_last_sent_vel
                if delta > step_max:
                    cmd_v = cyclic_last_sent_vel + step_max
                elif delta < -step_max:
                    cmd_v = cyclic_last_sent_vel - step_max
                else:
                    cmd_v = float(target_v)
                cyclic_last_sent_vel = cmd_v
                cmd_node.send_velocity(int(round(cmd_v)))
                cyclic_sent_value_text.set(
                    f"CSV下发: 物理目标 {target_v_mmps:.3f} mm/s -> 编码器 {int(round(cmd_v))} (目标 {int(round(target_v))})"
                )

        hz = _parse_float(rate_hz_var.get())
        if hz is None or hz <= 0.0:
            _cyclic_stop_internal(show_msg=False)
            status_send.set("周期下发停止：频率输入非法")
            return
        period_ms = max(1, int(1000.0 / hz))
        cyclic_after_id = root.after(period_ms, _cyclic_tick)

    def start_cyclic():
        nonlocal cyclic_running, cyclic_last_sent_pos, cyclic_last_sent_vel, current_cyclic_mode_code, cyclic_runner_proc
        mode_code = _cyclic_mode_code()
        hz = _parse_float(rate_hz_var.get())
        if mode_code not in (8, 9):
            status_send.set("周期模式输入非法（仅支持 8/9）")
            return
        if hz is None or hz <= 0.0:
            status_send.set("周期频率输入非法")
            return
        # 和标定互斥：两个地方都会用到 CSP，避免同时抢占话题
        _calib_stop_internal(show_msg=False)

        # 启动前做目标对齐/清零，避免旧目标造成突跳。
        # CSP 需要每次启动都对齐当前位置；CSV 保持原先“仅模式切换时清零”。
        if mode_code == 8:
            _align_targets_for_mode(mode_code)
        elif current_cyclic_mode_code != mode_code:
            _align_targets_for_mode(mode_code)

        # 仅使用 C++ 周期下发器：优先包安装前缀，其次回退常见路径
        this_file = os.path.realpath(__file__)
        ws_root = os.path.realpath(os.path.join(os.path.dirname(this_file), "..", "..", "..", ".."))
        candidate_paths = []
        if get_package_prefix is not None:
            try:
                pkg_prefix = get_package_prefix("mpk_ethercat_test")
                candidate_paths.append(
                    os.path.join(pkg_prefix, "lib", "mpk_ethercat_test", "cyclic_mode_runner")
                )
            except Exception:
                pass
        candidate_paths += [
            os.path.join(os.path.dirname(this_file), "cyclic_mode_runner"),
            os.path.join(ws_root, "install", "mpk_ethercat_test", "lib", "mpk_ethercat_test", "cyclic_mode_runner"),
            os.path.join(ws_root, "build", "mpk_ethercat_test", "cyclic_mode_runner"),
        ]
        runner_path = ""
        for p in candidate_paths:
            if os.path.isfile(p) and os.access(p, os.X_OK):
                runner_path = p
                break
        if not runner_path:
            status_send.set("启动外部周期下发失败: 未找到可执行文件 cyclic_mode_runner (C++)，请先 colcon build 并 source install/setup.bash")
            return

        mode_name = "csp" if mode_code == 8 else "csv"
        cmd = [
            runner_path,
            "--mode", mode_name,
            "--rate-hz", f"{hz}",
            "--topic", args.topic,
            "--control-word-topic", args.control_word_topic,
            "--op-mode-topic", args.op_mode_topic,
            "--position-topic", args.position_topic,
            "--velocity-topic", args.velocity_topic,
            "--require-enabled",
            "--require-mode",
            "--auto-enable",
        ]
        if mode_code == 8:
            csp_limit = _parse_float(csp_speed_limit_var.get())
            if csp_limit is None or csp_limit <= 0.0:
                status_send.set("CSP 上限(mm/s)输入非法")
                return
            physical_target = _clamp_csp_physical(float(csp_target_var.get()))
            target_enc = _physical_to_encoder_position(physical_target)
            if target_enc is None:
                status_send.set("启动失败：尚未获得有效bias，无法从物理量转换到编码器位置")
                return
            csp_limit_counts = _mmps_to_csp_countsps(csp_limit)
            cmd += [
                "--target", str(int(target_enc)),
                "--csp-speed", str(csp_limit_counts),
                "--csp-dynamic-target-topic", args.csp_dynamic_target_topic,
            ]
        else:
            target_v_mmps = _parse_float(vel_entry.get())
            if target_v_mmps is None:
                status_send.set("目标速度输入非法")
                return
            target_v = int(round(_mmps_to_csv_mrevs(target_v_mmps)))
            cmd += ["--target", str(target_v)]

        _cyclic_stop_internal(show_msg=False)
        try:
            cyclic_runner_proc = subprocess.Popen(cmd)
        except Exception as e:
            status_send.set(f"启动外部周期下发失败: {e}")
            return

        cyclic_running = True
        cyclic_last_sent_pos = None
        cyclic_last_sent_vel = None
        current_cyclic_mode_code = mode_code
        if mode_code == 8:
            cmd_node.send_csp_dynamic_target(int(target_enc))
        status_send.set(f"已启动外部周期下发: mode={mode_code}, rate={hz:.3f}Hz")

    def _calib_stop_internal(show_msg: bool = True):
        nonlocal calib_running, calib_after_id, calib_stage, calib_base_pos, calib_last_cmd_pos
        calib_running = False
        calib_stage = "idle"
        calib_base_pos = None
        calib_last_cmd_pos = None
        calib_sent_position_text.set("—")
        if calib_after_id is not None:
            root.after_cancel(calib_after_id)
            calib_after_id = None
        if show_msg:
            status_send.set("已停止标定流程")

    def _calib_tick():
        nonlocal calib_after_id, calib_stage, calib_last_cmd_pos
        if not calib_running:
            return

        state_str = _state_machine_str(latest_status_word) if latest_status_word is not None else "Unknown"
        hz = _parse_float(calib_rate_hz_var.get())
        if hz is None or hz <= 0.0:
            _calib_stop_internal(show_msg=False)
            status_send.set("标定停止：频率输入非法")
            return
        speed = _parse_float(calib_speed_var.get())
        if speed is None or speed <= 0.0:
            _calib_stop_internal(show_msg=False)
            status_send.set("标定停止：速度输入非法")
            return
        dt = 1.0 / hz
        period_ms = max(1, int(1000.0 / hz))

        # 阶段1：切到 CSP 并对齐当前位置，避免突跳
        if calib_stage == "prepare":
            cmd_node.send_op_mode(8)
            if calib_base_pos is not None:
                cmd_node.send_position(calib_base_pos)
                calib_sent_position_text.set(str(calib_base_pos))
            if latest_operation_mode == 8:
                calib_stage = "to_ready"

        # 阶段2：依赖系统已有自动状态切换到 Switched on，再由标定流程发送 0x000F
        elif calib_stage in ("to_ready", "to_switched_on", "to_enabled"):
            if state_str == "Operation enabled":
                calib_stage = "run_to_fault"
            elif state_str == "Switched on":
                cmd_node.send_control_word(0x000F)
                status_send.set("标定推进使能：检测到 Switched on，发送 0x000F")
            else:
                status_send.set(f"标定等待自动使能：当前状态 {state_str}")

        # 阶段3：按设定速度平滑向正方向推进，直到 Fault
        elif calib_stage == "run_to_fault":
            if calib_last_cmd_pos is None:
                if latest_position is not None:
                    calib_last_cmd_pos = int(round(latest_position))
                else:
                    calib_after_id = root.after(period_ms, _calib_tick)
                    return
            step = max(1, int(round(speed * dt)))
            cmd_pos = int(calib_last_cmd_pos) + step
            calib_last_cmd_pos = cmd_pos
            cmd_node.send_position(cmd_pos)
            calib_sent_position_text.set(str(cmd_pos))
            status_send.set(f"标定进行中：position={cmd_pos}, step={step}, rate={hz:.1f}Hz")

        calib_after_id = root.after(period_ms, _calib_tick)

    def start_calibration():
        nonlocal calib_running, calib_stage, calib_base_pos, calib_last_cmd_pos
        if latest_position is None:
            status_send.set("标定失败：尚未收到当前位置")
            return
        _cyclic_stop_internal(show_msg=False)
        _calib_stop_internal(show_msg=False)

        calib_base_pos = int(round(latest_position))
        calib_last_cmd_pos = calib_base_pos
        if latest_physical_position is not None:
            phys = _clamp_csp_physical(float(latest_physical_position))
            csp_target_var.set(phys)
            csp_target_text.set(f"{phys:.3f}")
        op_mode_var.set("8 (CSP)")

        calib_running = True
        calib_stage = "prepare"
        status_send.set("开始标定：切CSP并逐步使能，随后按设定速度向正方向平滑运动直到Fault")
        _calib_tick()

    btn_frame = tk.Frame(right)
    btn_frame.grid(row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, pad))

    tk.Button(btn_frame, text="发送 op_mode", width=14, command=send_op_mode).grid(row=0, column=0, padx=4, pady=4, sticky="w")
    tk.Button(btn_frame, text="发送位置(PP/CSP)", width=14, command=send_position).grid(row=0, column=1, padx=4, pady=4, sticky="w")
    tk.Button(btn_frame, text="发送 速度", width=14, command=send_velocity).grid(row=0, column=2, padx=4, pady=4, sticky="w")

    rrow += 1
    tk.Label(right, text="周期命令下发:", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=(pad, 6))
    rrow += 1

    tk.Label(right, text="周期模式:", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=4)
    tk.Label(right, text="跟随上方运行模式 op_mode（仅 8/9）", font=tkfont.Font(root, size=9), fg="gray", anchor="w").grid(
        row=rrow, column=1, sticky="w", padx=pad, pady=4
    )
    rrow += 1

    tk.Label(right, text="频率 Hz:", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=4)
    tk.Entry(right, width=18, textvariable=rate_hz_var).grid(row=rrow, column=1, sticky="w", padx=pad, pady=4)
    rrow += 1

    tk.Label(right, text="CSP上限(mm/s):", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=4)
    tk.Entry(right, width=18, textvariable=csp_speed_limit_var).grid(row=rrow, column=1, sticky="w", padx=pad, pady=4)
    rrow += 1

    tk.Label(right, text="CSV上限(mm/s²):", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=4)
    tk.Entry(right, width=18, textvariable=csv_acc_limit_var).grid(row=rrow, column=1, sticky="w", padx=pad, pady=4)
    rrow += 1

    tk.Label(right, textvariable=cyclic_sent_value_text, font=tkfont.Font(root, size=9), fg="gray", anchor="w", wraplength=340).grid(
        row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, 4)
    )
    rrow += 1

    cyclic_btn_frame = tk.Frame(right)
    cyclic_btn_frame.grid(row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(2, pad))
    tk.Button(cyclic_btn_frame, text="启动周期下发", width=14, command=start_cyclic).grid(row=0, column=0, padx=4, pady=4, sticky="w")
    tk.Button(cyclic_btn_frame, text="停止周期下发", width=14, command=_cyclic_stop_internal).grid(row=0, column=1, padx=4, pady=4, sticky="w")
    rrow += 1

    rrow += 1
    tk.Label(right, text="常用 Control Word:", font=label_font, anchor="w").grid(row=rrow, column=0, sticky="w", padx=pad, pady=(pad, 6))
    rrow += 1

    cw_frame = tk.Frame(right)
    cw_frame.grid(row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, pad))

    # CiA402 常用控制字（位组合）：0x0006 Shutdown, 0x0007 Switch on, 0x000F Enable operation,
    # 0x0000 Disable voltage, 0x0002 Quick stop
    tk.Button(cw_frame, text="Shutdown (0x0006)", width=18, command=lambda: send_control_word(0x0006, "Shutdown")).grid(row=0, column=0, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="Switch on/Disable (0x0007)", width=18, command=lambda: send_control_word(0x0007, "Switch on/disable")).grid(row=0, column=1, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="Enable (0x000F)", width=18, command=lambda: send_control_word(0x000F, "Enable operation")).grid(row=1, column=0, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="Disable Voltage (0x0000)", width=18, command=lambda: send_control_word(0x0000, "Disable Voltage")).grid(row=1, column=1, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="Quick stop (0x0002)", width=18, command=lambda: send_control_word(0x0002, "Quick stop")).grid(row=2, column=0, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="Legacy reset (0x0080)", width=18, command=lambda: send_control_word(0x0080, "Legacy fault reset")).grid(row=2, column=1, padx=4, pady=4, sticky="w")
    tk.Button(cw_frame, text="PP Start 脉冲", width=18, command=send_pp_start_pulse).grid(row=3, column=0, padx=4, pady=4, sticky="w")

    rrow += 1
    tk.Label(
        right,
        text=(
            f"发布:\n"
            f"- cw: {args.control_word_topic}\n"
            f"- op: {args.op_mode_topic}\n"
            f"- pos:{args.position_topic}\n"
            f"- vel:{args.velocity_topic}"
        ),
        font=tkfont.Font(root, size=9),
        fg="gray",
        justify="left",
        anchor="w",
    ).grid(row=rrow, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, pad))

    row += 1
    tk.Label(left, text="标定流程:", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=(pad, 6))
    row += 1
    tk.Label(left, text="流程: CSP->逐步使能->按速度平滑正向运动->Fault取bias", font=tkfont.Font(root, size=9), fg="gray", anchor="w").grid(
        row=row, column=0, columnspan=4, sticky="w", padx=pad, pady=2
    )
    row += 1
    tk.Label(left, text="标定频率 Hz:", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=2)
    tk.Entry(left, width=12, textvariable=calib_rate_hz_var).grid(row=row, column=1, sticky="w", padx=pad, pady=2)
    row += 1
    tk.Label(left, text="标定速度 count/s:", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=2)
    tk.Entry(left, width=12, textvariable=calib_speed_var).grid(row=row, column=1, sticky="w", padx=pad, pady=2)
    row += 1
    tk.Label(left, text="Position bias:", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=2)
    tk.Label(left, textvariable=position_bias_text, font=value_font, anchor="w", fg="#1a1a1a").grid(row=row, column=1, sticky="w", padx=pad, pady=2)
    row += 1
    tk.Label(left, text="标定下发position:", font=label_font, anchor="w").grid(row=row, column=0, sticky="w", padx=pad, pady=2)
    tk.Label(left, textvariable=calib_sent_position_text, font=value_font, anchor="w", fg="#1a1a1a").grid(row=row, column=1, sticky="w", padx=pad, pady=2)
    row += 1
    calib_btn_frame = tk.Frame(left)
    calib_btn_frame.grid(row=row, column=0, columnspan=4, sticky="w", padx=pad, pady=(2, pad))
    tk.Button(calib_btn_frame, text="开始标定", width=14, command=start_calibration).grid(row=0, column=0, padx=4, pady=4, sticky="w")
    tk.Button(calib_btn_frame, text="停止标定", width=14, command=_calib_stop_internal).grid(row=0, column=1, padx=4, pady=4, sticky="w")

    def on_closing():
        _cyclic_stop_internal(show_msg=False)
        _calib_stop_internal(show_msg=False)
        sub_node.destroy_node()
        cmd_node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    def spin_once():
        nonlocal cyclic_runner_proc
        rclpy.spin_once(sub_node, timeout_sec=0.0)
        rclpy.spin_once(cmd_node, timeout_sec=0.0)
        if cyclic_runner_proc is not None and cyclic_runner_proc.poll() is not None:
            code = cyclic_runner_proc.returncode
            cyclic_runner_proc = None
            _cyclic_stop_internal(show_msg=False)
            status_send.set(f"外部周期下发进程已退出，code={code}")
        root.after(50, spin_once)

    root.after(50, spin_once)
    root.mainloop()


if __name__ == "__main__":
    main()
