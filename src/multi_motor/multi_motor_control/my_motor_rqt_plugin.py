#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
my_motor_rqt_plugin.py
======================
rqt plugin (ROS 2 Humble + PyQt5) for a cluster of Denali XCR
CiA-402 drives sitting on the ICube ethercat_driver_ros2 backend.

Architecture
------------
    MotorRQTPlugin            <- rqt_gui_py.plugin.Plugin subclass
        |
        +-- MultiMotorWidget  <- QWidget, hosts N MotorPanel
        |        |
        |        +-- MotorPanel * N
        |
        +-- RosBridge         <- owns rclpy.Node, runs in its own
                                 thread, talks to Qt via signals
Topic / service summary
-----------------------
  /multi_motor/states_deg                (sub)  raw/restored position, velocity
  /dynamic_joint_states                  (sub)  status_word, mode disp
  /cia402_cmd_controller/commands        (pub)  Control Word array
  /cia402_mode_controller/commands       (pub)  Mode array
  /fault_reset_controller/commands       (pub)  reset_fault pulse
  /multi_motor/pp_position_deg/commands       (pub)  profile-position target
  /multi_motor/csp_position_deg/commands      (pub)  position setpoint
  /multi_motor/csv_velocity_deg_s/commands    (pub)  velocity setpoint
  /multi_motor/pv_velocity_deg_s/commands     (pub)  target velocity in PV mode
  /multi_motor/reset_encoder_restore          (pub)  reset restored encoder state
  /controller_manager/list_controllers   (srv)
  /controller_manager/switch_controller  (srv)
"""

from __future__ import annotations

import os
import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, SwitchController

from python_qt_binding.QtCore import Qt, QObject, QTimer, pyqtSignal, pyqtSlot
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QComboBox, QSlider, QLineEdit, QGroupBox,
    QTextEdit, QDoubleSpinBox, QSpinBox, QScrollArea,
)

from rqt_gui_py.plugin import Plugin

try:
    # When imported as part of the installed ROS 2 package
    #   (rqt finds us through plugin.xml -> <class type=...>)
    from multi_motor_control.cia402 import (
        CW, MODE, parse_status_word, next_control_word,
    )
    from multi_motor_control.joint_limits import (
        JointLimit, load_joint_limits, save_min_zero_calibration,
        save_range_calibration, save_linear_manual_calibration,
        DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV,
        DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV,
        DEFAULT_LINEAR_DIRECTION,
    )
except ImportError:
    # Running this file directly, e.g. `python3 my_motor_rqt_plugin.py`
    from cia402 import CW, MODE, parse_status_word, next_control_word
    from joint_limits import (
        JointLimit, load_joint_limits, save_min_zero_calibration,
        save_range_calibration, save_linear_manual_calibration,
        DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV,
        DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV,
        DEFAULT_LINEAR_DIRECTION,
    )


# ==========================================================
# ROS <-> Qt bridge
# ==========================================================
class RosBridge(QObject):
    """
    Owns an rclpy Node + a private executor in a background thread.
    Exposes Qt signals for UI thread consumption.

    Public Qt signals
    -----------------
      joint_state_updated(list[float] pos, list[float] vel,
                          list[float] raw_cnt,
                          list[float] restored_cnt,
                          list[float] restored_pos)
      dynamic_state_updated(list[int] status_words, list[int] mode_disp)
      controllers_updated(dict[str, str] name -> state)
      log_message(str level, str msg)
    """

    joint_state_updated   = pyqtSignal(list, list, list, list, list)
    dynamic_state_updated = pyqtSignal(list, list)
    controllers_updated   = pyqtSignal(dict)
    log_message           = pyqtSignal(str, str)

    def __init__(self, joint_names: List[str]):
        super().__init__()
        self._joint_names = joint_names
        self._num = len(joint_names)

        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node('multi_motor_rqt_plugin')
        self._setup_node()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        try:
            while not self._stop.is_set() and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            if self._node is not None:
                self._node.destroy_node()

    # ------------------------------------------------------
    # Node setup
    # ------------------------------------------------------
    def _setup_node(self):
        n = self._node

        # ---- publishers ----
        self._pub_cw   = n.create_publisher(
            Float64MultiArray, '/cia402_cmd_controller/commands', 10)
        self._pub_mode = n.create_publisher(
            Float64MultiArray, '/cia402_mode_controller/commands', 10)
        self._pub_rst  = n.create_publisher(
            Float64MultiArray, '/fault_reset_controller/commands', 10)

        self._pub_pp   = n.create_publisher(
            Float64MultiArray, '/multi_motor/pp_position_deg/commands', 10)
        self._pub_csp  = n.create_publisher(
            Float64MultiArray, '/multi_motor/csp_position_deg/commands', 10)
        self._pub_csp_counts = n.create_publisher(
            Float64MultiArray, '/csp_controller/commands', 10)
        self._pub_csv  = n.create_publisher(
            Float64MultiArray, '/multi_motor/csv_velocity_deg_s/commands', 10)
        self._pub_pv   = n.create_publisher(
            Float64MultiArray, '/multi_motor/pv_velocity_deg_s/commands', 10)
        self._pub_reset_encoder_restore = n.create_publisher(
            Float64MultiArray, '/multi_motor/reset_encoder_restore', 10)

        # ---- subscribers ----
        n.create_subscription(
            DynamicJointState, '/multi_motor/states_deg',
            self._cb_degree_state, 10)
        n.create_subscription(
            DynamicJointState, '/dynamic_joint_states',
            self._cb_dynamic_state, 10)

        # ---- service clients ----
        self._cli_list   = n.create_client(
            ListControllers,  '/controller_manager/list_controllers')
        self._cli_switch = n.create_client(
            SwitchController, '/controller_manager/switch_controller')

        # Periodically query controller states.
        n.create_timer(1.0, self._tick_list_controllers)

    # ------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------
    def _cb_degree_state(self, msg: DynamicJointState):
        pos = [0.0] * self._num
        vel = [0.0] * self._num
        raw_cnt = [math.nan] * self._num
        restored_cnt = [math.nan] * self._num
        restored_pos = [math.nan] * self._num
        for jn, ifv in zip(msg.joint_names, msg.interface_values):
            if jn not in self._joint_names:
                continue
            i = self._joint_names.index(jn)
            for name, val in zip(ifv.interface_names, ifv.values):
                if name in ('position_deg', 'position_mm'):
                    pos[i] = val
                elif name in ('velocity_deg_s', 'velocity_mm_s'):
                    vel[i] = val
                elif name == 'raw_position_cnt':
                    raw_cnt[i] = val
                elif name == 'restored_position_cnt':
                    restored_cnt[i] = val
                elif name in ('restored_position_deg', 'restored_position_mm'):
                    restored_pos[i] = val
            if not math.isfinite(restored_pos[i]):
                restored_pos[i] = pos[i]
        self.joint_state_updated.emit(
            pos, vel, raw_cnt, restored_cnt, restored_pos)

    def _cb_dynamic_state(self, msg: DynamicJointState):
        sw   = [0] * self._num
        mode = [0] * self._num
        for jn, ifv in zip(msg.joint_names, msg.interface_values):
            if jn not in self._joint_names:
                continue
            i = self._joint_names.index(jn)
            for name, val in zip(ifv.interface_names, ifv.values):
                if name == 'status_word':
                    sw[i] = int(val) & 0xFFFF
                elif name == 'modes_of_operation_display':
                    mode[i] = int(val)
        self.dynamic_state_updated.emit(sw, mode)

    def _tick_list_controllers(self):
        if not self._cli_list.service_is_ready():
            return
        fut = self._cli_list.call_async(ListControllers.Request())
        fut.add_done_callback(self._on_list_done)

    def _on_list_done(self, fut):
        try:
            resp = fut.result()
        except Exception as e:                      # noqa: BLE001
            self.log_message.emit('warn', f'list_controllers failed: {e}')
            return
        state_map = {c.name: c.state for c in resp.controller}
        self.controllers_updated.emit(state_map)

    # ------------------------------------------------------
    # Publish helpers -- called from the Qt thread, safe
    # ------------------------------------------------------
    def _publish_array(self, pub, vec: List[float]):
        if pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [float(x) for x in vec]
        pub.publish(msg)

    def publish_control_word(self, vec: List[float]):
        self._publish_array(self._pub_cw, vec)

    def publish_mode(self, vec: List[float]):
        self._publish_array(self._pub_mode, vec)

    def publish_fault_reset(self, vec: List[float]):
        self._publish_array(self._pub_rst, vec)

    def publish_pp(self, vec: List[float]):
        self._publish_array(self._pub_pp, vec)

    def publish_csp(self, vec: List[float]):
        self._publish_array(self._pub_csp, vec)

    def publish_csp_counts(self, vec: List[float]):
        self._publish_array(self._pub_csp_counts, vec)

    def publish_csv(self, vec: List[float]):
        self._publish_array(self._pub_csv, vec)

    def publish_pv(self, vec: List[float]):
        self._publish_array(self._pub_pv, vec)

    def publish_reset_encoder_restore(self, vec: List[float]):
        self._publish_array(self._pub_reset_encoder_restore, vec)

    # ------------------------------------------------------
    # switch_controller - fire-and-forget, result logged
    # ------------------------------------------------------
    def switch_controller(self, activate: List[str],
                          deactivate: List[str],
                          strictness: int = SwitchController.Request.BEST_EFFORT):
        if not self._cli_switch.service_is_ready():
            self.log_message.emit('error', 'switch_controller service not ready')
            return
        req = SwitchController.Request()
        req.activate_controllers   = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness             = strictness
        req.activate_asap          = True
        fut = self._cli_switch.call_async(req)

        def _done(f):
            try:
                ok = f.result().ok
            except Exception as e:                  # noqa: BLE001
                self.log_message.emit('error', f'switch_controller: {e}')
                return
            act = ','.join(activate) or '-'
            deact = ','.join(deactivate) or '-'
            lvl = 'info' if ok else 'warn'
            self.log_message.emit(lvl,
                f'switch_controller activate=[{act}] deactivate=[{deact}] ok={ok}')

        fut.add_done_callback(_done)


# ==========================================================
# Single-motor panel
# ==========================================================
class MotorPanel(QGroupBox):
    """
    UI controls for one motor. Emits no signals: the parent widget
    polls / pushes values via public methods.
    """

    MODES_UI = [
        ('PP  (Profile Position)',        MODE.PROFILE_POSITION),
        ('CSP (Cyclic Sync Position)', MODE.CSP),
        ('CSV (Cyclic Sync Velocity)', MODE.CSV),
        ('PV  (Profile Velocity)',     MODE.PROFILE_VELOCITY),
    ]

    # Fallback range if calibrate.json is not available.
    POS_RANGE_REVS = 5
    # 速度目标范围：±5 圈/秒，可按需放宽。
    VEL_RANGE_REVS_PER_S = 5
    POS_SLIDER_SCALE = 10  # slider integer step = 0.1 deg

    def __init__(self, joint_name: str, index: int,
                 encoder_resolution: float,
                 position_limit: Optional[JointLimit] = None,
                 fallback_unit: str = 'deg',
                 parent=None):
        super().__init__(joint_name, parent)
        self.joint_name = joint_name
        self.index = index
        self.encoder_resolution = encoder_resolution
        self.position_limit = position_limit
        self.position_unit = (
            position_limit.display_unit
            if position_limit is not None else fallback_unit)

        self._cached_status_word = 0
        self._cached_mode_disp   = 0
        self._cached_pos_deg = (
            position_limit.default_deg if position_limit is not None else 0.0)
        self._cached_raw_position_cnt = math.nan
        self._cached_restored_position_cnt = math.nan
        self._cached_restored_pos_deg = math.nan
        self._cached_vel_deg_s   = 0.0

        self._build_ui()

    # ------------------------------------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)

        # ---- Status grid -----------------------------
        s = QGridLayout()
        row = 0
        s.addWidget(QLabel('Mode:'), row, 0)
        self.lbl_mode = QLabel('-')
        s.addWidget(self.lbl_mode, row, 1)
        row += 1

        s.addWidget(QLabel('State:'), row, 0)
        self.lbl_state = QLabel('-')
        s.addWidget(self.lbl_state, row, 1)
        row += 1

        s.addWidget(QLabel('Status Word:'), row, 0)
        self.lbl_sw = QLabel('0x0000')
        s.addWidget(self.lbl_sw, row, 1)
        row += 1

        s.addWidget(QLabel('Raw Position:'), row, 0)
        self.lbl_pos = QLabel('n/a cnt')
        s.addWidget(self.lbl_pos, row, 1)
        row += 1

        s.addWidget(QLabel('Restored Position:'), row, 0)
        self.lbl_restored_pos = QLabel(f'n/a {self.position_unit}')
        s.addWidget(self.lbl_restored_pos, row, 1)
        row += 1

        s.addWidget(QLabel('Velocity:'), row, 0)
        self.lbl_vel = QLabel(f'0.000 {self.position_unit}/s')
        s.addWidget(self.lbl_vel, row, 1)

        gb_status = QGroupBox('Status')
        gb_status.setLayout(s)
        root.addWidget(gb_status)

        # ---- State machine buttons -------------------
        sm = QHBoxLayout()
        self.btn_enable = QPushButton('Enable')
        self.btn_disable = QPushButton('Disable')
        self.btn_reset_fault = QPushButton('Reset Fault')
        for b, color in ((self.btn_enable,      '#4CAF50'),
                         (self.btn_disable,     '#F44336'),
                         (self.btn_reset_fault, '#FF9800')):
            b.setStyleSheet(f'background-color:{color}; color:white;')
            sm.addWidget(b)
        root.addLayout(sm)

        # ---- Mode combo ------------------------------
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel('Target mode:'))
        self.cmb_mode = QComboBox()
        for label, _ in self.MODES_UI:
            self.cmb_mode.addItem(label)
        self.cmb_mode.setCurrentIndex(2)  # CSV: safe zero-velocity enable path
        mode_row.addWidget(self.cmb_mode, 1)
        self.btn_apply_mode = QPushButton('Apply')
        mode_row.addWidget(self.btn_apply_mode)
        root.addLayout(mode_row)

        # ---- CSP controls ----------------------------
        # 位置目标单位：角度。ROS 层 unit_converter 负责 deg -> counts。
        pos_min_deg, pos_max_deg = self._position_bounds()
        pos_min_slider = int(round(pos_min_deg * self.POS_SLIDER_SCALE))
        pos_max_slider = int(round(pos_max_deg * self.POS_SLIDER_SCALE))
        pos_default_deg = self._clamp_position_deg(
            self.feedback_position_deg())

        self.gb_csp = QGroupBox(
            f'PP / CSP - Position setpoint ({self.position_unit})')
        csp = QVBoxLayout(self.gb_csp)
        csp_row = QHBoxLayout()
        self.sld_pos = QSlider(Qt.Horizontal)
        self.sld_pos.setRange(pos_min_slider, pos_max_slider)
        self.sld_pos.setSingleStep(1)
        self.sld_pos.setPageStep(10)
        self.sld_pos.setValue(int(round(pos_default_deg * self.POS_SLIDER_SCALE)))
        csp_row.addWidget(self.sld_pos, 1)
        self.spn_pos = QDoubleSpinBox()
        self.spn_pos.setDecimals(3)
        self.spn_pos.setRange(pos_min_deg, pos_max_deg)
        self.spn_pos.setSingleStep(1.0)
        self.spn_pos.setSuffix(f' {self.position_unit}')
        self.spn_pos.setGroupSeparatorShown(True)
        self.spn_pos.setValue(pos_default_deg)
        csp_row.addWidget(self.spn_pos)
        csp.addLayout(csp_row)

        stream_row = QHBoxLayout()
        self.btn_stream_start = QPushButton('Start streaming')
        self.btn_stream_start.setStyleSheet('background-color:#2196F3; color:white;')
        self.btn_stream_stop = QPushButton('Stop streaming')
        self.btn_stream_stop.setEnabled(False)
        self.btn_send_pp = QPushButton('Send PP (one-shot)')
        self.btn_send_pp.setStyleSheet('background-color:#455A64; color:white;')
        stream_row.addWidget(self.btn_send_pp)
        stream_row.addWidget(self.btn_stream_start)
        stream_row.addWidget(self.btn_stream_stop)
        csp.addLayout(stream_row)
        root.addWidget(self.gb_csp)

        # ---- CSV / PV controls -----------------------
        # 速度目标单位跟随轴类型：旋转轴 deg/s，线性轴 mm/s。
        if self.position_unit == 'mm':
            screw_lead = (
                self.position_limit.screw_lead_mm_per_rev
                if self.position_limit is not None else
                DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV)
            vel_limit = self.VEL_RANGE_REVS_PER_S * screw_lead
            vel_step = 1.0
        else:
            vel_limit = self.VEL_RANGE_REVS_PER_S * 360.0
            vel_step = 10.0

        self.gb_vel = QGroupBox(
            f'CSV / PV - Velocity setpoint ({self.position_unit}/s)')
        vel = QHBoxLayout(self.gb_vel)
        self.spn_vel = QDoubleSpinBox()
        self.spn_vel.setDecimals(3)
        self.spn_vel.setRange(-vel_limit, vel_limit)
        self.spn_vel.setSingleStep(vel_step)
        self.spn_vel.setSuffix(f' {self.position_unit}/s')
        self.spn_vel.setGroupSeparatorShown(True)
        vel.addWidget(self.spn_vel, 1)
        self.btn_send_vel = QPushButton('Send (one-shot)')
        self.btn_send_vel.setStyleSheet('background-color:#9C27B0; color:white;')
        vel.addWidget(self.btn_send_vel)
        root.addWidget(self.gb_vel)

        # ---- slider/spin linkage ---------------------
        self.sld_pos.valueChanged.connect(
            lambda v: self.spn_pos.setValue(v / self.POS_SLIDER_SCALE))
        self.spn_pos.valueChanged.connect(
            lambda v: self.sld_pos.setValue(
                int(round(v * self.POS_SLIDER_SCALE))))

    def _position_bounds(self) -> tuple[float, float]:
        if self.position_limit is not None:
            return self.position_limit.min_deg, self.position_limit.max_deg
        span = self.POS_RANGE_REVS * 360.0
        return -span, span

    def _velocity_bounds(self) -> tuple[float, float, float]:
        if self.position_unit == 'mm':
            screw_lead = (
                self.position_limit.screw_lead_mm_per_rev
                if self.position_limit is not None else
                DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV)
            limit = self.VEL_RANGE_REVS_PER_S * screw_lead
            return -limit, limit, 1.0
        limit = self.VEL_RANGE_REVS_PER_S * 360.0
        return -limit, limit, 10.0

    def _clamp_position_deg(self, value_deg: float) -> float:
        min_deg, max_deg = self._position_bounds()
        return min(max(float(value_deg), min_deg), max_deg)

    def set_position_setpoint_deg(self, value_deg: float) -> float:
        target = self._clamp_position_deg(value_deg)
        self.sld_pos.blockSignals(True)
        self.spn_pos.blockSignals(True)
        try:
            self.sld_pos.setValue(int(round(target * self.POS_SLIDER_SCALE)))
            self.spn_pos.setValue(target)
        finally:
            self.sld_pos.blockSignals(False)
            self.spn_pos.blockSignals(False)
        return target

    def update_position_limit(self, position_limit: Optional[JointLimit]) -> None:
        self.position_limit = position_limit
        if position_limit is not None:
            self.position_unit = position_limit.display_unit
        pos_min_deg, pos_max_deg = self._position_bounds()
        vel_min, vel_max, vel_step = self._velocity_bounds()
        self.sld_pos.blockSignals(True)
        self.spn_pos.blockSignals(True)
        try:
            self.sld_pos.setRange(
                int(round(pos_min_deg * self.POS_SLIDER_SCALE)),
                int(round(pos_max_deg * self.POS_SLIDER_SCALE)))
            self.spn_pos.setRange(pos_min_deg, pos_max_deg)
            self.spn_pos.setSuffix(f' {self.position_unit}')
            self.spn_vel.setRange(vel_min, vel_max)
            self.spn_vel.setSingleStep(vel_step)
            self.spn_vel.setSuffix(f' {self.position_unit}/s')
            self.gb_csp.setTitle(
                f'PP / CSP - Position setpoint ({self.position_unit})')
            self.gb_vel.setTitle(
                f'CSV / PV - Velocity setpoint ({self.position_unit}/s)')
        finally:
            self.sld_pos.blockSignals(False)
            self.spn_pos.blockSignals(False)
        self.set_position_setpoint_deg(self.feedback_position_deg())

    # ------------------------------------------------------
    # public getters
    # ------------------------------------------------------
    def selected_mode(self) -> int:
        return self.MODES_UI[self.cmb_mode.currentIndex()][1]

    def position_setpoint_deg(self) -> float:
        """PP/CSP 目标位置，单位：deg。"""
        return self._clamp_position_deg(self.spn_pos.value())

    def velocity_setpoint_deg_s(self) -> float:
        """CSV/PV 目标速度，单位：deg/s。"""
        return float(self.spn_vel.value())

    def feedback_position_deg(self) -> float:
        """Current feedback position in calibrated degree space.

        Prefer restored position so GUI setpoints stay consistent with the
        power-loss recovery model. Fall back to raw calibrated position before
        restored feedback is available.
        """
        if math.isfinite(self._cached_restored_pos_deg):
            return float(self._cached_restored_pos_deg)
        return float(self._cached_pos_deg)

    # ------------------------------------------------------
    # alignment helpers
    # ------------------------------------------------------
    def align_setpoint_to_feedback(self) -> float:
        """
        指令初始化对齐：把 PP/CSP 的位置 spinbox/slider 强制设置为
        当前恢复后的反馈位置；如果配置了 calibrate.json，
        则该目标值会被限制在标定的 min_deg/max_deg 范围内。

        必须在以下两个时机调用：
          1. 切到 CSP 模式（Apply）后：让用户看到的滑条停在「当前点」，
             避免后续误操作把电机拽回零位。
          2. 开始 CSP streaming 前：保证 _tick_csp_stream 发出的
             第一帧位置指令 = 当前反馈位置，避免电机阶跃。

        返回对齐后的目标值（deg）。
        """
        return self.set_position_setpoint_deg(self.feedback_position_deg())

    # ------------------------------------------------------
    # public updaters (called from UI thread via MultiMotorWidget)
    # ------------------------------------------------------
    def update_status(self, status_word: int, mode_disp: int):
        self._cached_status_word = status_word
        self._cached_mode_disp   = mode_disp
        st = parse_status_word(status_word)
        self.lbl_sw.setText(f'0x{status_word:04X}')
        self.lbl_state.setText(st.state +
                               ('  [FAULT]' if st.fault else ''))
        self.lbl_mode.setText(MODE.NAMES.get(mode_disp, f'mode {mode_disp}'))

        # Button gating based on state
        self.btn_enable.setEnabled(not st.fault and not st.operation_enabled)
        self.btn_disable.setEnabled(st.operation_enabled or st.switched_on
                                    or st.ready_to_switch_on)
        self.btn_reset_fault.setEnabled(st.fault)

    @staticmethod
    def _format_count(value: float) -> str:
        if not math.isfinite(value):
            return 'n/a cnt'
        return f'{int(round(value)):+d} cnt'

    def _format_position(self, value: float) -> str:
        if not math.isfinite(value):
            return f'n/a {self.position_unit}'
        if self.position_unit == 'mm':
            return f'{value:+.3f} mm'
        revs = value / 360.0
        return f'{value:+.3f} deg   ({revs:+.3f} rev)'

    def update_joint_state(
            self,
            pos_deg: float,
            vel_deg_s: float,
            raw_position_cnt: float,
            restored_position_cnt: float,
            restored_pos_deg: float):
        """
        /multi_motor/states_deg 由 unit_converter 从底层 counts 换算而来。
        GUI 额外显示 raw cnt 和恢复后的物理位置。
        """
        self._cached_pos_deg = float(pos_deg)
        self._cached_raw_position_cnt = float(raw_position_cnt)
        self._cached_restored_position_cnt = float(restored_position_cnt)
        self._cached_restored_pos_deg = float(restored_pos_deg)
        self._cached_vel_deg_s = float(vel_deg_s)
        self.lbl_pos.setText(self._format_count(self._cached_raw_position_cnt))
        self.lbl_restored_pos.setText(
            self._format_position(self._cached_restored_pos_deg))
        if self.position_unit == 'mm':
            self.lbl_vel.setText(f'{vel_deg_s:+.3f} mm/s')
        else:
            rps = vel_deg_s / 360.0
            self.lbl_vel.setText(f'{vel_deg_s:+.3f} deg/s ({rps:+.3f} rev/s)')

    @property
    def status_word(self) -> int:
        return self._cached_status_word

    @property
    def mode_display(self) -> int:
        return self._cached_mode_disp


# ==========================================================
# Main widget
# ==========================================================
class MultiMotorWidget(QWidget):
    """
    Hosts N MotorPanel. Orchestrates ROS traffic and the CiA 402
    state machine.
    """

    # Default layout, can be overridden at construction time.
    DEFAULT_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    DEFAULT_ENCODER_RES = 865075.2   # 2^17 * 6.6 gearbox
    LINEAR_AXIS_INDEX = 0

    # Mode -> motion-controller name
    MODE_CTRL = {
        MODE.PROFILE_POSITION:  'pp_controller',
        MODE.CSP:              'csp_controller',
        MODE.CSV:              'csv_controller',
        MODE.PROFILE_VELOCITY: 'pv_controller',
    }
    ALL_MOTION_CTRLS = (
        'pp_controller', 'csp_controller', 'csv_controller', 'pv_controller')

    # Streaming period for CSP
    CSP_STREAM_HZ = 100
    STAGGER_BATCH_SIZE = 1
    STAGGER_DELAY_MS = 600

    CALIBRATION_VELOCITY_DEG_S = 8.0
    CALIBRATION_STALL_VEL_DEG_S = 1.0
    CALIBRATION_STALL_SECONDS = 1.5
    CALIBRATION_TARGET_LEAD_DEG = 3.0
    CALIBRATION_FAULT_TARGET_LEAD_DEG = 0.5
    CALIBRATION_STALL_PROGRESS_DEG = 0.2
    CALIBRATION_MIN_RUN_SECONDS = 1.0
    CALIBRATION_MIN_MOVEMENT_DEG = 1.0
    CALIBRATION_NO_MOVE_ABORT_SECONDS = 3.0
    CALIBRATION_TIMEOUT_SECONDS = 45.0
    CALIBRATION_RANGE_TIMEOUT_SECONDS = 240.0

    def __init__(self,
                 joint_names: Optional[List[str]] = None,
                 encoder_resolution: float = DEFAULT_ENCODER_RES,
                 parent=None):
        super().__init__(parent)
        self.joint_names = joint_names or self.DEFAULT_JOINTS
        self.num = len(self.joint_names)
        self.encoder_resolution = encoder_resolution
        self.position_limits, self._limits_path = load_joint_limits()

        # State
        self._active_ctrl: Optional[str] = None
        self._streaming_csp = [False] * self.num
        self._csv_targets = [0.0] * self.num
        self._pv_targets = [0.0] * self.num
        self._enable_all_timers: list[QTimer] = []
        self._calibrating_zero = False
        self._calibrating_range = False
        self._calibrating_linear_axis = False
        self._calib_start_time = 0.0
        self._calib_stall_since: list[Optional[float]] = [None] * self.num
        self._calib_stall_raw_at = [0.0] * self.num
        self._calib_hit = [False] * self.num
        self._calib_raw_max: dict[str, float] = {}
        self._calib_raw_min: dict[str, float] = {}
        self._calib_range_phase = ['negative'] * self.num
        self._calib_phase_start_time = [0.0] * self.num
        self._calib_start_raw = [0.0] * self.num
        self._calib_target_raw = [0.0] * self.num
        self._calib_last_tick = 0.0
        self._calib_indices: list[int] = []
        self._linear_calib_min_cnt = math.nan
        self._linear_calib_max_cnt = math.nan
        self._linear_calib_last_log_time = 0.0

        # ROS bridge
        self.bridge = RosBridge(self.joint_names)
        self.bridge.joint_state_updated.connect(self._on_joint_state)
        self.bridge.dynamic_state_updated.connect(self._on_dynamic_state)
        self.bridge.controllers_updated.connect(self._on_controllers)
        self.bridge.log_message.connect(self._on_log)

        # UI
        self._build_ui()
        if self._limits_path is not None:
            self._log(f'Loaded joint limits from {self._limits_path}')
        else:
            self._log('No calibrate.json found; using fallback position range.',
                      'warn')

        # CSP streaming timer
        self._csp_timer = QTimer(self)
        self._csp_timer.setInterval(int(1000 / self.CSP_STREAM_HZ))
        self._csp_timer.timeout.connect(self._tick_csp_stream)

        self._calib_timer = QTimer(self)
        self._calib_timer.setInterval(50)
        self._calib_timer.timeout.connect(self._tick_calibration)

        # Enable-sequence timer (CiA 402 3-step walk)
        self._enable_timers: dict[int, QTimer] = {}

        # Kick off ROS
        self.bridge.start()

        # Safety: as soon as we have the publisher ready, flood
        # Control Word = 0 so EcCiA402Drive does not auto-enable.
        QTimer.singleShot(500,
            lambda: self.bridge.publish_control_word([0.0] * self.num))

    # ------------------------------------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.addWidget(QLabel('<h2>Denali XCR Multi-Motor Control</h2>'))

        # Calibration toolbar
        bar = QHBoxLayout()
        self.btn_enable_all = QPushButton('Enable All')
        self.btn_enable_all.setStyleSheet(
            'background:#2E7D32; color:white; font-weight:bold; padding:6px;')
        self.btn_disable_all = QPushButton('Disable All')
        self.btn_disable_all.setStyleSheet(
            'background:#C62828; color:white; font-weight:bold; padding:6px;')
        self.btn_calibrate_linear = QPushButton('Calibrate Linear M1')
        self.btn_calibrate_linear.setStyleSheet(
            'background:#5D4037; color:white; font-weight:bold; padding:6px;')
        self.btn_calibrate_zero = QPushButton('Calibrate Rotary Zero M2-M5')
        self.btn_calibrate_zero.setStyleSheet(
            'background:#00695C; color:white; font-weight:bold; padding:6px;')
        self.btn_calibrate_range = QPushButton('Calibrate Rotary Range M2-M5')
        self.btn_calibrate_range.setStyleSheet(
            'background:#0277BD; color:white; font-weight:bold; padding:6px;')
        self.btn_go_default = QPushButton('Go Default')
        self.btn_go_default.setStyleSheet(
            'background:#455A64; color:white; font-weight:bold; padding:6px;')
        self.btn_stop_calibrate = QPushButton('Stop Calibrate')
        self.btn_stop_calibrate.setEnabled(False)
        self.btn_stop_calibrate.setStyleSheet(
            'background:#6D4C41; color:white; font-weight:bold; padding:6px;')
        bar.addWidget(self.btn_enable_all)
        bar.addWidget(self.btn_disable_all)
        bar.addWidget(self.btn_calibrate_linear)
        bar.addWidget(self.btn_calibrate_zero)
        bar.addWidget(self.btn_calibrate_range)
        bar.addWidget(self.btn_go_default)
        bar.addWidget(self.btn_stop_calibrate)
        bar.addStretch(1)
        root.addLayout(bar)
        self.btn_enable_all.clicked.connect(
            lambda: self._start_enable_all_staggered())
        self.btn_disable_all.clicked.connect(lambda: self._disable_all())
        self.btn_calibrate_linear.clicked.connect(
            self._toggle_linear_axis_calibration)
        self.btn_calibrate_zero.clicked.connect(self._start_min_zero_calibration)
        self.btn_calibrate_range.clicked.connect(
            self._start_range_calibration)
        self.btn_go_default.clicked.connect(lambda: self._go_default())
        self.btn_stop_calibrate.clicked.connect(
            lambda: self._stop_calibration())

        # Panel grid (scrollable so 3->8 motors still fits)
        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        self.panels: List[MotorPanel] = []
        for i, jn in enumerate(self.joint_names):
            p = MotorPanel(
                jn, i, self.encoder_resolution,
                position_limit=self.position_limits.get(jn),
                fallback_unit=self._default_unit_for_index(i))
            p.btn_enable.clicked.connect(
                lambda _, idx=i: self._start_enable_sequence(idx))
            p.btn_disable.clicked.connect(
                lambda _, idx=i: self._on_disable(idx))
            p.btn_reset_fault.clicked.connect(
                lambda _, idx=i: self._on_reset_fault(idx))
            p.btn_apply_mode.clicked.connect(
                lambda _, idx=i: self._on_apply_mode(idx))
            p.btn_send_pp.clicked.connect(
                lambda _, idx=i: self._on_send_pp(idx))
            p.btn_stream_start.clicked.connect(
                lambda _, idx=i: self._on_stream_start(idx))
            p.btn_stream_stop.clicked.connect(
                lambda _, idx=i: self._on_stream_stop(idx))
            p.btn_send_vel.clicked.connect(
                lambda _, idx=i: self._on_send_vel(idx))
            grid.addWidget(p, i // 2, i % 2)
            self.panels.append(p)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_host)
        root.addWidget(scroll, 1)

        # Controller-state footer
        ctrl = QGroupBox('Controller states')
        ctrl_lay = QHBoxLayout(ctrl)
        self.lbl_ctrl = {n: QLabel(f'{n}: ?') for n in
            ('joint_state_broadcaster', 'cia402_cmd_controller',
             'cia402_mode_controller', 'fault_reset_controller',
             'pp_controller', 'csp_controller', 'csv_controller',
             'pv_controller')}
        for w in self.lbl_ctrl.values():
            ctrl_lay.addWidget(w)
        root.addWidget(ctrl)

        # Log
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(120)
        root.addWidget(self.txt_log)

    # ======================================================
    # Subscription -> UI
    # ======================================================
    @pyqtSlot(list, list, list, list, list)
    def _on_joint_state(self, pos, vel, raw_cnt, restored_cnt, restored_pos):
        for i, p in enumerate(self.panels):
            p.update_joint_state(
                pos[i], vel[i], raw_cnt[i], restored_cnt[i], restored_pos[i])

    @pyqtSlot(list, list)
    def _on_dynamic_state(self, sw, mode):
        for i, p in enumerate(self.panels):
            p.update_status(sw[i], mode[i])

    @pyqtSlot(dict)
    def _on_controllers(self, state_map: dict):
        for name, lbl in self.lbl_ctrl.items():
            st = state_map.get(name, 'not loaded')
            lbl.setText(f'{name}: {st}')
            color = {'active': 'green', 'inactive': 'orange'}.get(st, 'gray')
            lbl.setStyleSheet(f'color:{color};')

    @pyqtSlot(str, str)
    def _on_log(self, level, msg):
        self._log(msg, level)

    # ======================================================
    # Action handlers
    # ======================================================

    def _linear_axis_index(self) -> Optional[int]:
        if 0 <= self.LINEAR_AXIS_INDEX < self.num:
            return self.LINEAR_AXIS_INDEX
        return None

    def _rotary_motor_indices(self) -> list[int]:
        linear_idx = self._linear_axis_index()
        return [i for i in range(self.num) if i != linear_idx]

    def _default_unit_for_index(self, idx: int) -> str:
        return 'mm' if idx == self.LINEAR_AXIS_INDEX else 'deg'

    def _panel_unit(self, idx: int) -> str:
        return self.panels[idx].position_unit

    def _format_panel_value(self, idx: int, value: float) -> str:
        unit = self._panel_unit(idx)
        return f'{value:+.3f} {unit}'

    def _clamp_position(self, joint_name: str, value_deg: float) -> float:
        limit = self.position_limits.get(joint_name)
        if limit is None:
            return float(value_deg)
        return limit.clamp(value_deg)

    def _clamp_position_vector(self, vec: List[float]) -> List[float]:
        return [
            self._clamp_position(joint_name, value)
            for joint_name, value in zip(self.joint_names, vec)
        ]

    def _feedback_position_vector(self) -> List[float]:
        return self._clamp_position_vector([
            panel.feedback_position_deg() for panel in self.panels
        ])

    def _raw_position_from_feedback(self, idx: int) -> float:
        joint_name = self.joint_names[idx]
        limit = self.position_limits.get(joint_name)
        if limit is not None and limit.is_linear:
            raw_count = float(self.panels[idx]._cached_raw_position_cnt)
            return raw_count if math.isfinite(raw_count) else 0.0
        pos = float(self.panels[idx]._cached_pos_deg)
        if limit is None:
            return pos
        return limit.calibrated_to_raw(pos)

    def _rotary_encoder_resolution(self, joint_name: str) -> float:
        limit = self.position_limits.get(joint_name)
        if limit is not None and not limit.is_linear:
            return limit.encoder_resolution
        return self.encoder_resolution

    def _raw_deg_to_counts(
            self, value_deg: float, joint_name: Optional[str] = None) -> float:
        resolution = (
            self._rotary_encoder_resolution(joint_name)
            if joint_name is not None else self.encoder_resolution)
        counts = int(round(float(value_deg) * resolution / 360.0))
        return float(counts)

    def _raw_position_counts_vector(self, raw_vec: List[float]) -> List[float]:
        counts: List[float] = []
        for i, value in enumerate(raw_vec):
            limit = self.position_limits.get(self.joint_names[i])
            if limit is not None and limit.is_linear:
                counts.append(float(value))
            else:
                counts.append(self._raw_deg_to_counts(
                    value, self.joint_names[i]))
        return counts

    def _current_drive_counts_vector(self) -> Optional[List[float]]:
        counts: List[float] = []
        for panel in self.panels:
            raw_count = float(panel._cached_raw_position_cnt)
            if not math.isfinite(raw_count):
                self._log(
                    'Current raw encoder feedback is not ready; cannot build '
                    'a safe hold-position command.',
                    'warn')
                return None
            counts.append(raw_count)
        return counts

    def _drive_count_for_restored_position(
            self, idx: int, target_value: float) -> Optional[float]:
        panel = self.panels[idx]
        raw_count = float(panel._cached_raw_position_cnt)
        restored_count = float(panel._cached_restored_position_cnt)
        if not math.isfinite(raw_count) or not math.isfinite(restored_count):
            self._log(
                f'{self.joint_names[idx]}: restored encoder feedback is not '
                'ready; cannot convert default pose safely.',
                'warn')
            return None

        limit = self.position_limits.get(self.joint_names[idx])
        if limit is None:
            restored_target = int(round(
                float(target_value) *
                self._rotary_encoder_resolution(self.joint_names[idx]) /
                360.0))
        else:
            restored_target = int(round(
                limit.calibrated_to_raw_counts(
                    target_value, self.encoder_resolution)))
        restore_offset = restored_count - raw_count
        return float(restored_target - restore_offset)

    def _switch_motion_controller(self, target_ctrl: str) -> None:
        self.bridge.switch_controller(
            activate=[target_ctrl],
            deactivate=[c for c in self.ALL_MOTION_CTRLS if c != target_ctrl],
            strictness=SwitchController.Request.BEST_EFFORT,
        )
        self._active_ctrl = target_ctrl

    def _reload_position_limits_for_gui(self) -> None:
        self.position_limits, self._limits_path = load_joint_limits()
        for panel in self.panels:
            panel.update_position_limit(
                self.position_limits.get(panel.joint_name))

    def _stop_csp_streaming_controls(self) -> None:
        self._csp_timer.stop()
        self._streaming_csp = [False] * self.num
        for panel in self.panels:
            panel.btn_stream_start.setEnabled(True)
            panel.btn_stream_stop.setEnabled(False)

    def _cancel_enable_all_timers(self) -> None:
        for timer in self._enable_all_timers:
            timer.stop()
        self._enable_all_timers.clear()

    def _start_enable_all_staggered(self) -> None:
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            self._log('Enable All: stop calibration first.', 'warn')
            return

        self._cancel_enable_all_timers()
        pending: list[int] = []
        faulted: list[str] = []
        already_enabled: list[str] = []
        for i, panel in enumerate(self.panels):
            st = parse_status_word(panel.status_word)
            if st.fault:
                faulted.append(self.joint_names[i])
            elif st.operation_enabled:
                already_enabled.append(self.joint_names[i])
            else:
                pending.append(i)

        if faulted:
            self._log(
                f'Enable All: skip faulted joints; reset them first: {faulted}',
                'warn')
        if already_enabled:
            self._log(
                f'Enable All: already enabled: {already_enabled}')
        if not pending:
            self._log('Enable All: no joints need enabling.')
            return

        self._log(
            f'Enable All: enabling {len(pending)} joints, '
            f'{self.STAGGER_BATCH_SIZE} per {self.STAGGER_DELAY_MS} ms.')
        batches = [
            pending[i:i + self.STAGGER_BATCH_SIZE]
            for i in range(0, len(pending), self.STAGGER_BATCH_SIZE)
        ]
        for batch_index, batch in enumerate(batches):
            timer = QTimer(self)
            timer.setSingleShot(True)

            def _run_batch(batch=batch, timer=timer):
                if timer in self._enable_all_timers:
                    self._enable_all_timers.remove(timer)
                for idx in batch:
                    self._start_enable_sequence(idx)

            timer.timeout.connect(_run_batch)
            self._enable_all_timers.append(timer)
            timer.start(batch_index * self.STAGGER_DELAY_MS)

    def _disable_all(self) -> None:
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            self._stop_calibration(reason='disabled')
        self._cancel_enable_all_timers()
        for timer in self._enable_timers.values():
            timer.stop()
        self._enable_timers.clear()
        self._stop_csp_streaming_controls()
        self.bridge.publish_csv([0.0] * self.num)
        self.bridge.publish_pv([0.0] * self.num)
        self._csv_targets = [0.0] * self.num
        self._pv_targets = [0.0] * self.num
        self.bridge.publish_control_word(
            [float(CW.DISABLE_VOLTAGE)] * self.num)
        self._log('Disable All: sent Disable Voltage to every joint.', 'warn')

    def _default_position_vector(
            self, indices: Optional[list[int]] = None) -> List[float]:
        defaults = self._feedback_position_vector()
        target_indices = range(self.num) if indices is None else indices
        for i in target_indices:
            joint_name = self.joint_names[i]
            limit = self.position_limits.get(joint_name)
            defaults[i] = limit.default_deg if limit is not None else 0.0
        return self._clamp_position_vector(defaults)

    def _go_default(self) -> None:
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            self._log('Go Default: stop calibration first.', 'warn')
            return
        target_indices = self._rotary_motor_indices()
        if self._enabled_calibration_not_ready('Go Default', target_indices):
            return

        self._reload_position_limits_for_gui()
        default_vec = self._default_position_vector(target_indices)
        current_counts = self._current_drive_counts_vector()
        if current_counts is None:
            return
        default_counts = list(current_counts)
        for idx in target_indices:
            target_count = self._drive_count_for_restored_position(
                idx, default_vec[idx])
            if target_count is None:
                return
            default_counts[idx] = target_count

        self._stop_csp_streaming_controls()
        for idx in target_indices:
            self.panels[idx].set_position_setpoint_deg(default_vec[idx])

        self.bridge.publish_csv([0.0] * self.num)
        self.bridge.publish_pv([0.0] * self.num)
        self.bridge.publish_csp_counts(current_counts)
        self._switch_motion_controller('csp_controller')

        def _publish_default():
            self.bridge.publish_csp_counts(default_counts)
            mvec = [float(p.mode_display or MODE.NO_MODE)
                    for p in self.panels]
            for idx in target_indices:
                mvec[idx] = float(MODE.CSP)
            self.bridge.publish_mode(mvec)
            self._log(
                'Go Default: M2-M5 CSP targets -> '
                f'{[self._format_panel_value(i, default_vec[i]) for i in target_indices]}.')

        QTimer.singleShot(250, _publish_default)
        QTimer.singleShot(
            400, lambda: self.bridge.publish_csp_counts(default_counts))

    # ---- Enable (direct mode) ----
    def _start_enable_sequence(self, idx: int):
        """
        使能流程（CSP 安全策略）：

        CSP 模式的关键安全问题：
        当 mode_of_operation_display == CSP 时，EtherCAT 驱动会直接使用
        position command_interface 的值写入 0x607A。如果此时 command_interface
        里有陈旧的值（来自上一次运行或未初始化），电机会瞬间飞到那个位置。

        解决方案：CSP 模式使用 CSV 引导
          1. 先用 CSV 模式使能（CSV 不看 0x607A，安全）
          2. Op Enabled 后激活 csp_controller
          3. 发送当前反馈位置到 csp_controller，等待生效
          4. 确认 command_interface 值正确后，再切到 CSP 模式
        """
        # 取消正在跑的旧 walk
        t = self._enable_timers.get(idx)
        if t is not None:
            t.stop()

        user_mode = self.panels[idx].selected_mode()
        if user_mode == MODE.NO_MODE:
            user_mode = MODE.CSV

        self._log(
            f'{self.joint_names[idx]}: enable sequence start '
            f'(target={MODE.NAMES.get(user_mode, user_mode)})')

        # Position modes use CSV as a zero-velocity enable path. This keeps
        # stale 0x607A targets out of the CiA 402 enable transition.
        enable_mode = (
            MODE.CSV if user_mode in (MODE.PROFILE_POSITION, MODE.CSP)
            else user_mode)

        if enable_mode == MODE.CSV:
            self._csv_targets = [0.0] * self.num
            self.bridge.publish_csv(self._csv_targets)
            self._switch_motion_controller('csv_controller')
            self._log(
                f'{self.joint_names[idx]}: CSV controller active with '
                'zero velocity before CiA 402 switch-on')

        # ---- Step 1: 注入使能模式 ----
        mvec = [float(p.mode_display or MODE.NO_MODE) for p in self.panels]
        mvec[idx] = float(enable_mode)
        self.bridge.publish_mode(mvec)
        self._log(
            f'{self.joint_names[idx]}: injected enable mode '
            f'{MODE.NAMES.get(enable_mode, enable_mode)}')

        # ---- Step 2: position modes pre-align the visible setpoint ----
        if user_mode in (MODE.PROFILE_POSITION, MODE.CSP):
            cur = self.panels[idx].align_setpoint_to_feedback()
            self._log(
                f'{self.joint_names[idx]}: position setpoint '
                f'aligned to feedback @ '
                f'{self._format_panel_value(idx, cur)}')

        # ---- Step 3+4: 延迟 300 ms 等 mode 生效，再走状态机 ----
        QTimer.singleShot(
            300, lambda: self._walk_to_operation_enabled(
                idx, enable_mode, user_mode))

    def _walk_to_operation_enabled(self, idx: int,
                                   enable_mode: int, user_mode: int):
        """
        50 ms / tick 的 CiA 402 状态机 walk。
        到达 Operation Enabled 后，激活对应的 motion controller。
        如果 user_mode != enable_mode（CSP 引导），后续会安全切换。
        """
        timer = QTimer(self)
        timer.setInterval(50)
        ticks = {'n': 0}

        def step():
            ticks['n'] += 1
            sw = self.panels[idx].status_word
            st = parse_status_word(sw)

            # 每拍重发 mode 向量，保持使能模式
            mvec = [float(p.mode_display or MODE.NO_MODE) for p in self.panels]
            mvec[idx] = float(enable_mode)
            self.bridge.publish_mode(mvec)

            cw = next_control_word(sw)
            vec = self._current_cw_vector()
            vec[idx] = float(cw)
            self.bridge.publish_control_word(vec)

            if ticks['n'] % 10 == 0:
                self._log(
                    f'{self.joint_names[idx]}: enabling '
                    f'({MODE.NAMES.get(enable_mode, enable_mode)}) … '
                    f'tick={ticks["n"]} sw=0x{sw:04X} '
                    f'state={st.state} cw=0x{int(cw):04X}')

            if st.operation_enabled:
                self._log(
                    f'{self.joint_names[idx]}: Operation Enabled ✓ '
                    f'(mode={MODE.NAMES.get(enable_mode, enable_mode)})')
                timer.stop()
                # 使能成功，激活对应的 motion controller
                self._activate_motion_controller(idx, user_mode)
            elif ticks['n'] > 60:
                self._log(
                    f'{self.joint_names[idx]}: enable timeout '
                    f'(sw=0x{sw:04X}, state={st.state})', 'warn')
                timer.stop()

        timer.timeout.connect(step)
        self._enable_timers[idx] = timer
        timer.start()

    def _activate_motion_controller(self, idx: int, target_mode: int):
        """
        Op Enabled 后激活 motion controller 并安全切到目标模式。

        CSP 安全时序（防止飞车）：
          1. 先激活 csp_controller（此时还在 CSV 模式，override_command=true，
             驱动用 default_value=last_position_ 写 0x607A，安全）
          2. 发送当前反馈位置到 csp_controller topic
          3. 等 200ms 让 command_interface 稳定到正确值
          4. 再注入 CSP 模式（override_command=false，此时 command_interface
             已经是正确的当前位置，不会飞车）
        """
        target_ctrl = self.MODE_CTRL.get(target_mode)
        if target_ctrl is None:
            self._log(
                f'{self.joint_names[idx]}: no motion controller for mode '
                f'{target_mode}', 'warn')
            return

        # ---- Step 1: 激活 controller（此时仍在安全模式） ----
        self._switch_motion_controller(target_ctrl)
        self._log(
            f'{self.joint_names[idx]}: activated {target_ctrl}')

        # ---- Step 2: CSP 模式特殊处理 ----
        if target_mode == MODE.PROFILE_POSITION:
            cur = self.panels[idx].align_setpoint_to_feedback()

            def _publish_and_switch_pp():
                pos_vec = self._feedback_position_vector()
                self.bridge.publish_pp(pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: PP position = '
                    f'{self._format_panel_value(idx, cur)}, waiting for CI '
                    'to stabilize...')

                def _inject_pp_mode():
                    fresh_pos = self._feedback_position_vector()
                    self.bridge.publish_pp(fresh_pos)
                    mvec = [float(p.mode_display or MODE.NO_MODE)
                            for p in self.panels]
                    mvec[idx] = float(MODE.PROFILE_POSITION)
                    self.bridge.publish_mode(mvec)
                    self._log(
                        f'{self.joint_names[idx]}: PP mode injected safely')

                QTimer.singleShot(200, _inject_pp_mode)

            QTimer.singleShot(50, _publish_and_switch_pp)

        if target_mode == MODE.CSP:
            # 发送当前反馈位置，覆盖 command_interface 中可能的陈旧值
            cur = self.panels[idx].align_setpoint_to_feedback()
            cur_pos_vec = self._feedback_position_vector()

            def _publish_and_switch():
                # 连续发送位置确保 controller 收到
                self.bridge.publish_csp(cur_pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: CSP position = '
                    f'{self._format_panel_value(idx, cur)}, waiting for CI '
                    'to stabilize...')

                def _inject_csp_mode():
                    # 再发一次位置确保万无一失
                    fresh_pos = self._feedback_position_vector()
                    self.bridge.publish_csp(fresh_pos)
                    # 现在注入 CSP 模式
                    mvec = [float(p.mode_display or MODE.NO_MODE)
                            for p in self.panels]
                    mvec[idx] = float(MODE.CSP)
                    self.bridge.publish_mode(mvec)
                    self._log(
                        f'{self.joint_names[idx]}: CSP mode injected '
                        f'(CI should be safe now)')

                # 等 200ms 让 command_interface 稳定
                QTimer.singleShot(200, _inject_csp_mode)

            # 等 50ms 让 controller 完成激活
            QTimer.singleShot(50, _publish_and_switch)

    def _on_disable(self, idx: int):
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            self._stop_calibration(reason='disabled')
        self._log(f'{self.joint_names[idx]}: disable')
        t = self._enable_timers.pop(idx, None)
        if t is not None:
            t.stop()
        vec = self._current_cw_vector()
        vec[idx] = float(CW.DISABLE_VOLTAGE)
        self.bridge.publish_control_word(vec)

    def _tick_calibration(self):
        if self._calibrating_zero:
            self._tick_zero_calibration()
        elif self._calibrating_range:
            self._tick_range_calibration()
        elif self._calibrating_linear_axis:
            self._tick_linear_axis_calibration()

    def _stop_calibration(self, reason: str = 'stopped'):
        if self._calibrating_zero:
            self._finish_zero_calibration(save=False, reason=reason)
        elif self._calibrating_range:
            self._finish_range_calibration(save=False, reason=reason)
        elif self._calibrating_linear_axis:
            self._finish_linear_axis_calibration(save=False, reason=reason)

    def _enabled_calibration_not_ready(
            self, label: str,
            indices: Optional[list[int]] = None) -> list[str]:
        not_ready = []
        target_indices = range(self.num) if indices is None else indices
        for i in target_indices:
            p = self.panels[i]
            st = parse_status_word(p.status_word)
            if not st.operation_enabled:
                not_ready.append(self.joint_names[i])
        if not_ready:
            self._log(
                f'{label}: enable each joint first: {not_ready}', 'warn')
        return not_ready

    def _enter_raw_csp_calibration(self, label: str) -> None:
        self._csp_timer.stop()
        self._streaming_csp = [False] * self.num
        for p in self.panels:
            p.btn_stream_start.setEnabled(True)
            p.btn_stream_stop.setEnabled(False)

        self.bridge.publish_csv([0.0] * self.num)
        self.bridge.publish_pv([0.0] * self.num)
        self.bridge.publish_csp_counts(
            self._raw_position_counts_vector(self._calib_target_raw))
        self._switch_motion_controller('csp_controller')
        self._log(
            f'{label}: using CSP raw-position commands; stop immediately if '
            'the mechanism binds unexpectedly.', 'warn')

    def _set_calibration_buttons_active(self, active: bool) -> None:
        self.btn_enable_all.setEnabled(not active)
        self.btn_calibrate_zero.setEnabled(not active)
        self.btn_calibrate_range.setEnabled(not active)
        self.btn_calibrate_linear.setEnabled(
            not active or self._calibrating_linear_axis)
        self.btn_go_default.setEnabled(not active)
        self.btn_stop_calibrate.setEnabled(active)

    # ---- Manual linear-axis calibration ----------------------
    def _linear_axis_scale(self) -> tuple[float, float, float]:
        idx = self._linear_axis_index()
        limit = (
            self.position_limits.get(self.joint_names[idx])
            if idx is not None else None)
        if limit is not None and limit.is_linear:
            return (
                limit.encoder_counts_per_rev,
                limit.screw_lead_mm_per_rev,
                limit.linear_direction,
            )
        return (
            DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV,
            DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV,
            DEFAULT_LINEAR_DIRECTION,
        )

    def _linear_axis_feedback_count(self) -> float:
        idx = self._linear_axis_index()
        if idx is None:
            return math.nan
        panel = self.panels[idx]
        if math.isfinite(panel._cached_restored_position_cnt):
            return float(panel._cached_restored_position_cnt)
        return float(panel._cached_raw_position_cnt)

    def _linear_axis_range_mm(self) -> float:
        encoder_counts_per_rev, screw_lead_mm_per_rev, _ = (
            self._linear_axis_scale())
        if (
            not math.isfinite(self._linear_calib_min_cnt) or
            not math.isfinite(self._linear_calib_max_cnt)
        ):
            return math.nan
        return (
            abs(self._linear_calib_max_cnt - self._linear_calib_min_cnt) *
            screw_lead_mm_per_rev / encoder_counts_per_rev
        )

    def _linear_axis_zero_count(self) -> float:
        _, _, linear_direction = self._linear_axis_scale()
        if linear_direction < 0.0:
            return self._linear_calib_max_cnt
        return self._linear_calib_min_cnt

    def _toggle_linear_axis_calibration(self):
        if self._calibrating_linear_axis:
            self._finish_linear_axis_calibration(save=True, reason='saved')
        else:
            self._start_linear_axis_calibration()

    def _start_linear_axis_calibration(self):
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            return
        if self._limits_path is None:
            self._log('Linear M1 calibration: no calibrate.json path found.',
                      'error')
            return
        idx = self._linear_axis_index()
        if idx is None:
            self._log('Linear M1 calibration: joint_1 is not available.',
                      'error')
            return

        count = self._linear_axis_feedback_count()
        if not math.isfinite(count):
            self._log(
                'Linear M1 calibration: waiting for encoder feedback first.',
                'warn')
            return

        self._calibrating_linear_axis = True
        self._calib_start_time = time.monotonic()
        self._calib_last_tick = self._calib_start_time
        self._linear_calib_min_cnt = count
        self._linear_calib_max_cnt = count
        self._linear_calib_last_log_time = 0.0
        self.btn_calibrate_linear.setText('Save Linear M1')
        self._set_calibration_buttons_active(True)
        self._calib_timer.start()
        self._log(
            'Linear M1 calibration: recording encoder range only; move M1 '
            'manually to both ends, then click "Save Linear M1".',
            'warn')

    def _tick_linear_axis_calibration(self):
        if not self._calibrating_linear_axis:
            return
        count = self._linear_axis_feedback_count()
        if not math.isfinite(count):
            return
        self._linear_calib_min_cnt = min(self._linear_calib_min_cnt, count)
        self._linear_calib_max_cnt = max(self._linear_calib_max_cnt, count)

        now = time.monotonic()
        if now - self._linear_calib_last_log_time < 1.0:
            return
        self._linear_calib_last_log_time = now
        self._log(
            'Linear M1 calibration: observed count range '
            f'[{self._linear_calib_min_cnt:.0f}, '
            f'{self._linear_calib_max_cnt:.0f}] -> '
            f'{self._linear_axis_range_mm():.3f} mm.')

    def _finish_linear_axis_calibration(self, save: bool, reason: str):
        if not self._calibrating_linear_axis:
            return
        if save:
            range_mm = self._linear_axis_range_mm()
            if not math.isfinite(range_mm) or range_mm <= 0.0:
                self._log(
                    'Linear M1 calibration: move the axis through a '
                    'non-zero range before saving.',
                    'warn')
                return

            idx = self._linear_axis_index()
            joint_name = self.joint_names[idx]
            encoder_counts_per_rev, screw_lead_mm_per_rev, linear_direction = (
                self._linear_axis_scale())
            zero_count = self._linear_axis_zero_count()
            try:
                save_linear_manual_calibration(
                    self._limits_path,
                    joint_name,
                    zero_count,
                    range_mm,
                    encoder_counts_per_rev=encoder_counts_per_rev,
                    screw_lead_mm_per_rev=screw_lead_mm_per_rev,
                    linear_direction=linear_direction)
                self.position_limits, self._limits_path = load_joint_limits()
                self.panels[idx].update_position_limit(
                    self.position_limits.get(joint_name))
                self.panels[idx].set_position_setpoint_deg(0.0)
            except Exception as exc:                 # noqa: BLE001
                self._log(
                    f'Linear M1 calibration: failed to save {joint_name}: '
                    f'{exc}',
                    'error')
                return

            self._log(
                f'Linear M1 calibration: saved 0..{range_mm:.3f} mm '
                f'to {self._limits_path}; zero_count={zero_count:.0f}.')
        else:
            self._log(
                f'Linear M1 calibration: {reason}; no changes saved.',
                'warn')

        self._calib_timer.stop()
        self._calibrating_linear_axis = False
        self.btn_calibrate_linear.setText('Calibrate Linear M1')
        self._set_calibration_buttons_active(False)

    # ---- Initial zero calibration ----------------------
    def _start_min_zero_calibration(self):
        """
        Slowly move every enabled joint in the negative direction until its
        feedback velocity stalls, then write that raw minimum as calibrated
        zero.

        Stall-based homing is only an inference. A physical limit switch,
        drive torque/current threshold, or CiA 402 homing mode is safer.
        """
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            return
        if self._limits_path is None:
            self._log(
                'Rotary Zero M2-M5: no calibrate.json path found.', 'error')
            return

        target_indices = self._rotary_motor_indices()
        if self._enabled_calibration_not_ready(
                'Rotary Zero M2-M5', target_indices):
            return

        self._calibrating_zero = True
        self._calib_indices = target_indices
        self._calib_start_time = time.monotonic()
        self._calib_last_tick = self._calib_start_time
        self._calib_stall_since = [None] * self.num
        self._calib_stall_raw_at = [0.0] * self.num
        self._calib_hit = [i not in target_indices for i in range(self.num)]
        self._calib_raw_min = {}
        self._calib_raw_max = {}
        self._calib_start_raw = [
            self._raw_position_from_feedback(i) for i in range(self.num)]
        self._calib_target_raw = list(self._calib_start_raw)

        # CSP calibration bypasses the degree API because the unhomed
        # calibrated range may already be clamped.
        self._enter_raw_csp_calibration('Rotary Zero M2-M5')

        self._log(
            'Rotary Zero M2-M5: moving M2-M5 negative slowly.', 'warn')

        def _inject_csp_and_start():
            if not self._calibrating_zero:
                return
            self.bridge.publish_csp_counts(
                self._raw_position_counts_vector(self._calib_target_raw))
            mvec = [float(p.mode_display or MODE.NO_MODE)
                    for p in self.panels]
            for idx in target_indices:
                mvec[idx] = float(MODE.CSP)
            self.bridge.publish_mode(mvec)
            self._set_calibration_buttons_active(True)
            self._calib_last_tick = time.monotonic()
            self._calib_timer.start()

        QTimer.singleShot(300, _inject_csp_and_start)

    def _complete_joint_zero_calibration(
            self, idx: int, raw_min: float, detail: str) -> bool:
        joint_name = self.joint_names[idx]
        if self._calib_hit[idx]:
            return True

        self._calib_hit[idx] = True
        self._calib_raw_min[joint_name] = raw_min
        self._calib_target_raw[idx] = raw_min

        try:
            save_min_zero_calibration(
                self._limits_path, [joint_name], {joint_name: raw_min})
            self.position_limits, self._limits_path = load_joint_limits()
            self.panels[idx].update_position_limit(
                self.position_limits.get(joint_name))
            self.panels[idx].set_position_setpoint_deg(0.0)
            reset_vec = [math.nan] * self.num
            reset_vec[idx] = self._raw_deg_to_counts(raw_min, joint_name)
            self.bridge.publish_reset_encoder_restore(reset_vec)
        except Exception as exc:                     # noqa: BLE001
            self._log(
                f'Rotary Zero M2-M5: failed to save {joint_name}: {exc}',
                'error')
            self._finish_zero_calibration(save=False, reason='save failed')
            return False

        self._log(
            f'Rotary Zero M2-M5: {joint_name} saved min zero at '
            f'raw={raw_min:+.3f} deg ({detail}); remaining='
            f'{[self.joint_names[i] for i in self._calib_indices if not self._calib_hit[i]]}')
        return True

    def _tick_zero_calibration(self):
        if not self._calibrating_zero:
            return

        now = time.monotonic()
        elapsed = now - self._calib_start_time
        if elapsed > self.CALIBRATION_TIMEOUT_SECONDS:
            self._finish_zero_calibration(save=False, reason='timeout')
            return

        dt = max(0.0, min(now - self._calib_last_tick, 0.2))
        self._calib_last_tick = now
        any_moved_negative = False
        for i, p in enumerate(self.panels):
            if i not in self._calib_indices:
                continue
            if self._calib_hit[i]:
                any_moved_negative = True
                continue

            st = parse_status_word(p.status_word)
            raw_pos = self._raw_position_from_feedback(i)
            moved = self._calib_start_raw[i] - raw_pos
            target_lead = raw_pos - self._calib_target_raw[i]
            moved_negative = moved > 0.0
            any_moved_negative = any_moved_negative or moved_negative

            if st.fault:
                limit_like_fault = (
                    target_lead >= self.CALIBRATION_FAULT_TARGET_LEAD_DEG and
                    abs(float(p._cached_vel_deg_s)) <=
                    self.CALIBRATION_STALL_VEL_DEG_S)
                if limit_like_fault:
                    self._log(
                        f'Rotary Zero M2-M5: {self.joint_names[i]} faulted at '
                        f'near-zero velocity; treating as min limit. '
                        f'sw=0x{p.status_word:04X}, raw={raw_pos:+.3f} deg, '
                        f'moved={moved:+.3f} deg, '
                        f'lead={target_lead:+.3f} deg, '
                        f'vel={p._cached_vel_deg_s:+.3f} deg/s.',
                        'warn')
                    if not self._complete_joint_zero_calibration(
                            i, raw_pos, 'fault near limit'):
                        return
                    continue

                self._log(
                    f'Rotary Zero M2-M5: {self.joint_names[i]} faulted; abort. '
                    f'sw=0x{p.status_word:04X}, raw={raw_pos:+.3f} deg, '
                    f'moved={moved:+.3f} deg, '
                    f'lead={target_lead:+.3f} deg, '
                    f'vel={p._cached_vel_deg_s:+.3f} deg/s.',
                    'error')
                self._finish_zero_calibration(save=False, reason='fault')
                return

            target_is_ahead = target_lead >= self.CALIBRATION_TARGET_LEAD_DEG
            if target_is_ahead:
                if self._calib_stall_since[i] is None:
                    self._calib_stall_since[i] = now
                    self._calib_stall_raw_at[i] = raw_pos
                elif abs(raw_pos - self._calib_stall_raw_at[i]) > self.CALIBRATION_STALL_PROGRESS_DEG:
                    self._calib_stall_since[i] = now
                    self._calib_stall_raw_at[i] = raw_pos
                elif (now - self._calib_stall_since[i] >=
                      self.CALIBRATION_STALL_SECONDS):
                    raw_min = self._raw_position_from_feedback(i)
                    self._log(
                        f'Rotary Zero M2-M5: {self.joint_names[i]} '
                        f'stalled at raw {raw_min:+.3f} deg '
                        f'(lead={target_lead:+.3f} deg)')
                    if not self._complete_joint_zero_calibration(
                            i, raw_min, 'target lead stall'):
                        return
                    continue
            else:
                self._calib_stall_since[i] = None

            self._calib_target_raw[i] -= self.CALIBRATION_VELOCITY_DEG_S * dt

        self.bridge.publish_csp_counts(
            self._raw_position_counts_vector(self._calib_target_raw))

        if (elapsed > self.CALIBRATION_NO_MOVE_ABORT_SECONDS and
                not any_moved_negative):
            self._finish_zero_calibration(save=False, reason='no movement')
            return

        if all(self._calib_hit[i] for i in self._calib_indices):
            self._finish_zero_calibration(save=True, reason='complete')

    def _finish_zero_calibration(self, save: bool, reason: str):
        self._calib_timer.stop()
        hold_raw = list(self._calib_target_raw)
        self.bridge.publish_csp_counts(
            self._raw_position_counts_vector(hold_raw))
        self.bridge.publish_csv([0.0] * self.num)
        self._csv_targets = [0.0] * self.num
        self._calibrating_zero = False
        self._set_calibration_buttons_active(False)

        if not save:
            saved = [
                self.joint_names[i]
                for i in self._calib_indices if self._calib_hit[i]
            ]
            self._log(
                f'Rotary Zero M2-M5: {reason}; stopped. '
                f'Completed joints already saved: {saved}',
                'warn')
            return

        self._log(
            f'Rotary Zero M2-M5: complete; saved offsets to {self._limits_path}.')

    # ---- Full travel range calibration ----------------------
    def _start_range_calibration(self):
        """
        Move negative to find the minimum physical limit, then move positive
        to find the maximum physical limit. The measured difference becomes
        range_deg. The calibrated zero is left unchanged; the software max is
        updated to match the measured range, while the default pose is kept as
        an explicit user calibration.
        """
        if (self._calibrating_zero or self._calibrating_range or
                self._calibrating_linear_axis):
            return
        if self._limits_path is None:
            self._log(
                'Rotary Range M2-M5: no calibrate.json path found.', 'error')
            return

        target_indices = self._rotary_motor_indices()
        if self._enabled_calibration_not_ready(
                'Rotary Range M2-M5', target_indices):
            return

        now = time.monotonic()
        self._calibrating_range = True
        self._calib_indices = target_indices
        self._calib_start_time = now
        self._calib_last_tick = now
        self._calib_stall_since = [None] * self.num
        self._calib_stall_raw_at = [0.0] * self.num
        self._calib_hit = [i not in target_indices for i in range(self.num)]
        self._calib_raw_min = {}
        self._calib_raw_max = {}
        self._calib_range_phase = ['negative'] * self.num
        self._calib_phase_start_time = [now] * self.num
        self._calib_start_raw = [
            self._raw_position_from_feedback(i) for i in range(self.num)]
        self._calib_target_raw = list(self._calib_start_raw)

        self._enter_raw_csp_calibration('Rotary Range M2-M5')
        self._log(
            'Rotary Range M2-M5: moving M2-M5 negative first, then positive '
            'after each joint reaches its negative limit.', 'warn')

        def _inject_csp_and_start():
            if not self._calibrating_range:
                return
            self.bridge.publish_csp_counts(
                self._raw_position_counts_vector(self._calib_target_raw))
            mvec = [float(p.mode_display or MODE.NO_MODE)
                    for p in self.panels]
            for idx in target_indices:
                mvec[idx] = float(MODE.CSP)
            self.bridge.publish_mode(mvec)
            self._set_calibration_buttons_active(True)
            self._calib_last_tick = time.monotonic()
            self._calib_timer.start()

        QTimer.singleShot(300, _inject_csp_and_start)

    def _complete_joint_range_limit(
            self, idx: int, raw_limit: float, detail: str) -> bool:
        joint_name = self.joint_names[idx]
        phase = self._calib_range_phase[idx]
        now = time.monotonic()

        if phase == 'negative':
            self._calib_raw_min[joint_name] = raw_limit
            self._calib_range_phase[idx] = 'positive'
            self._calib_start_raw[idx] = raw_limit
            self._calib_target_raw[idx] = raw_limit
            self._calib_stall_since[idx] = None
            self._calib_stall_raw_at[idx] = raw_limit
            self._calib_phase_start_time[idx] = now
            self._log(
                f'Rotary Range M2-M5: {joint_name} found negative limit at '
                f'raw={raw_limit:+.3f} deg ({detail}); reversing positive.')
            return True

        if self._calib_hit[idx]:
            return True

        raw_min = self._calib_raw_min.get(joint_name)
        if raw_min is None:
            self._log(
                f'Rotary Range M2-M5: {joint_name} has no recorded negative '
                'limit; abort.', 'error')
            self._finish_range_calibration(save=False, reason='missing min')
            return False

        raw_max = raw_limit
        measured_range = raw_max - raw_min
        if measured_range <= 0.0:
            self._log(
                f'Rotary Range M2-M5: {joint_name} invalid range '
                f'min={raw_min:+.3f}, max={raw_max:+.3f}; abort.', 'error')
            self._finish_range_calibration(save=False, reason='invalid range')
            return False

        self._calib_hit[idx] = True
        self._calib_raw_max[joint_name] = raw_max
        self._calib_target_raw[idx] = raw_max

        try:
            save_range_calibration(
                self._limits_path,
                [joint_name],
                {joint_name: raw_min},
                {joint_name: raw_max})
            self.position_limits, self._limits_path = load_joint_limits()
            limit = self.position_limits.get(joint_name)
            self.panels[idx].update_position_limit(limit)
            if limit is not None:
                self.panels[idx].set_position_setpoint_deg(limit.default_deg)
        except Exception as exc:                     # noqa: BLE001
            self._log(
                f'Rotary Range M2-M5: failed to save {joint_name}: {exc}',
                'error')
            self._finish_range_calibration(save=False, reason='save failed')
            return False

        self._log(
            f'Rotary Range M2-M5: {joint_name} saved range='
            f'{measured_range:.3f} deg, raw_min={raw_min:+.3f} deg, '
            f'raw_max={raw_max:+.3f} deg ({detail}); remaining='
            f'{[self.joint_names[i] for i in self._calib_indices if not self._calib_hit[i]]}')
        return True

    def _tick_range_calibration(self):
        if not self._calibrating_range:
            return

        now = time.monotonic()
        elapsed = now - self._calib_start_time
        if elapsed > self.CALIBRATION_RANGE_TIMEOUT_SECONDS:
            self._finish_range_calibration(save=False, reason='timeout')
            return

        dt = max(0.0, min(now - self._calib_last_tick, 0.2))
        self._calib_last_tick = now
        active_count = 0
        all_active_no_move_timed_out = True

        for i, p in enumerate(self.panels):
            if i not in self._calib_indices:
                continue
            if self._calib_hit[i]:
                continue

            active_count += 1
            phase = self._calib_range_phase[i]
            direction = -1.0 if phase == 'negative' else 1.0
            phase_label = 'negative' if direction < 0.0 else 'positive'
            phase_elapsed = now - self._calib_phase_start_time[i]
            st = parse_status_word(p.status_word)
            raw_pos = self._raw_position_from_feedback(i)
            moved = direction * (raw_pos - self._calib_start_raw[i])
            target_lead = direction * (self._calib_target_raw[i] - raw_pos)
            moved_in_direction = moved >= self.CALIBRATION_MIN_MOVEMENT_DEG

            if moved_in_direction or (
                    phase_elapsed <= self.CALIBRATION_NO_MOVE_ABORT_SECONDS):
                all_active_no_move_timed_out = False

            if st.fault:
                limit_like_fault = (
                    phase_elapsed >= self.CALIBRATION_MIN_RUN_SECONDS and
                    moved_in_direction and
                    target_lead >= self.CALIBRATION_FAULT_TARGET_LEAD_DEG and
                    abs(float(p._cached_vel_deg_s)) <=
                    self.CALIBRATION_STALL_VEL_DEG_S)
                if limit_like_fault and direction > 0.0:
                    self._log(
                        f'Rotary Range M2-M5: {self.joint_names[i]} faulted at '
                        f'near-zero velocity; treating as positive limit. '
                        f'sw=0x{p.status_word:04X}, raw={raw_pos:+.3f} deg, '
                        f'moved={moved:+.3f} deg, '
                        f'lead={target_lead:+.3f} deg, '
                        f'vel={p._cached_vel_deg_s:+.3f} deg/s.',
                        'warn')
                    if not self._complete_joint_range_limit(
                            i, raw_pos, 'fault near positive limit'):
                        return
                    continue

                self._log(
                    f'Rotary Range M2-M5: {self.joint_names[i]} faulted during '
                    f'{phase_label} travel; abort. '
                    f'sw=0x{p.status_word:04X}, raw={raw_pos:+.3f} deg, '
                    f'moved={moved:+.3f} deg, lead={target_lead:+.3f} deg, '
                    f'vel={p._cached_vel_deg_s:+.3f} deg/s.',
                    'error')
                self._finish_range_calibration(save=False, reason='fault')
                return

            if (phase_elapsed >= self.CALIBRATION_MIN_RUN_SECONDS and
                    moved_in_direction):
                target_is_ahead = (
                    target_lead >= self.CALIBRATION_TARGET_LEAD_DEG)
                if target_is_ahead:
                    if self._calib_stall_since[i] is None:
                        self._calib_stall_since[i] = now
                        self._calib_stall_raw_at[i] = raw_pos
                    elif abs(raw_pos - self._calib_stall_raw_at[i]) > self.CALIBRATION_STALL_PROGRESS_DEG:
                        self._calib_stall_since[i] = now
                        self._calib_stall_raw_at[i] = raw_pos
                    elif (now - self._calib_stall_since[i] >=
                          self.CALIBRATION_STALL_SECONDS):
                        raw_limit = self._raw_position_from_feedback(i)
                        self._log(
                            f'Rotary Range M2-M5: {self.joint_names[i]} '
                            f'{phase_label} stall at raw '
                            f'{raw_limit:+.3f} deg '
                            f'(lead={target_lead:+.3f} deg)')
                        if not self._complete_joint_range_limit(
                                i, raw_limit,
                                f'{phase_label} target lead stall'):
                            return
                        continue
                else:
                    self._calib_stall_since[i] = None

            self._calib_target_raw[i] += (
                direction * self.CALIBRATION_VELOCITY_DEG_S * dt)

        self.bridge.publish_csp_counts(
            self._raw_position_counts_vector(self._calib_target_raw))

        if active_count > 0 and all_active_no_move_timed_out:
            self._finish_range_calibration(save=False, reason='no movement')
            return

        if all(self._calib_hit[i] for i in self._calib_indices):
            self._finish_range_calibration(save=True, reason='complete')

    def _finish_range_calibration(self, save: bool, reason: str):
        self._calib_timer.stop()
        hold_raw = list(self._calib_target_raw)
        self.bridge.publish_csp_counts(
            self._raw_position_counts_vector(hold_raw))
        self.bridge.publish_csv([0.0] * self.num)
        self._csv_targets = [0.0] * self.num
        self._calibrating_range = False
        self._set_calibration_buttons_active(False)

        if not save:
            saved = [
                self.joint_names[i]
                for i in self._calib_indices if self._calib_hit[i]
            ]
            mins = [
                self.joint_names[i] for i in self._calib_indices
                if (
                    self.joint_names[i] in self._calib_raw_min and
                    self.joint_names[i] not in saved
                )
            ]
            self._log(
                f'Rotary Range M2-M5: {reason}; stopped. '
                f'Completed joints already saved: {saved}; '
                f'joints with only negative limit recorded: {mins}',
                'warn')
            return

        self._log(
            f'Rotary Range M2-M5: complete; saved ranges to {self._limits_path}.')

    def _on_reset_fault(self, idx: int):
        """
        Reset a CiA 402 fault on joint `idx` and walk it back to Operation Enabled.

        Root-cause notes
        ----------------
        1. ec_update() priority: command_interface (non-NaN) > default_value.
           cia402_cmd_controller keeps writing 0.0 so the plugin's
           default_value (with bit-7 set) is never applied.
           Fix: write CW=0x0080 directly via cia402_cmd_controller.

        2. Denali XCR refuses Switch On (CW=0x0007) when
           modes_of_operation (0x6060) = 0 (No Mode), and refuses
           Enable Operation in CSP mode if 0x607a is far from 0x6064
           (Following Error). Fix: 由 _start_enable_sequence 统一处理
           mode 注入 + 位置对齐 + 等待 + walk。
        """
        self._log(f'{self.joint_names[idx]}: reset fault (direct CW=0x0080)')

        # Step 1 – fault_reset pulse on the dedicated interface (belt-and-suspenders)
        hi = [0.0] * self.num
        hi[idx] = 1.0
        lo = [0.0] * self.num
        self.bridge.publish_fault_reset(hi)
        QTimer.singleShot(100, lambda: self.bridge.publish_fault_reset(lo))

        # Step 2 – write CW=0x0080 directly via cia402_cmd_controller
        vec_reset = self._current_cw_vector()
        vec_reset[idx] = float(CW.FAULT_RESET)   # 0x0080
        self.bridge.publish_control_word(vec_reset)

        # Step 3 – 200 ms 后调用统一的 _start_enable_sequence，由它
        # 负责 mode 注入、CSP 位置对齐、再延迟 300 ms 启动 walk。
        # 总用时 ~500 ms，与之前一致，但逻辑只在一处维护。
        QTimer.singleShot(200, lambda: self._start_enable_sequence(idx))

    # ---- Mode switch (correct ordering) ----
    def _on_apply_mode(self, idx: int):
        target_mode = self.panels[idx].selected_mode()
        target_ctrl = self.MODE_CTRL[target_mode]
        self._log(f'{self.joint_names[idx]}: apply mode '
                  f'{MODE.NAMES[target_mode]} ({target_ctrl})')

        if target_mode == MODE.PROFILE_POSITION:
            cur = self.panels[idx].align_setpoint_to_feedback()
            self._log(f'{self.joint_names[idx]}: PP setpoint '
                      f'aligned to feedback '
                      f'{self._format_panel_value(idx, cur)}')
            self._switch_motion_controller(target_ctrl)
            self._log(f'{self.joint_names[idx]}: activated {target_ctrl}')

            def _publish_and_inject_pp():
                pos_vec = self._feedback_position_vector()
                self.bridge.publish_pp(pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: published PP position, '
                    f'waiting for CI to stabilize...')

                def _inject_pp():
                    fresh = self._feedback_position_vector()
                    self.bridge.publish_pp(fresh)
                    mvec = [float(p.mode_display or MODE.NO_MODE)
                            for p in self.panels]
                    mvec[idx] = float(target_mode)
                    self.bridge.publish_mode(mvec)
                    self._log(
                        f'{self.joint_names[idx]}: PP mode injected safely')

                QTimer.singleShot(200, _inject_pp)

            QTimer.singleShot(50, _publish_and_inject_pp)
            return

        if target_mode == MODE.CSP:
            # CSP 安全切换时序：
            # 1. 先激活 csp_controller（还在旧模式，override_command=true，安全）
            # 2. 发送当前位置到 controller
            # 3. 等 command_interface 稳定
            # 4. 最后再注入 CSP 模式
            cur = self.panels[idx].align_setpoint_to_feedback()
            self._log(f'{self.joint_names[idx]}: CSP setpoint '
                      f'aligned to feedback '
                      f'{self._format_panel_value(idx, cur)}')

            self._switch_motion_controller(target_ctrl)
            self._log(f'{self.joint_names[idx]}: activated {target_ctrl}')

            def _publish_and_inject():
                pos_vec = self._feedback_position_vector()
                self.bridge.publish_csp(pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: published CSP position, '
                    f'waiting for CI to stabilize...')

                def _inject():
                    fresh = self._feedback_position_vector()
                    self.bridge.publish_csp(fresh)
                    mvec = [float(p.mode_display or MODE.NO_MODE)
                            for p in self.panels]
                    mvec[idx] = float(target_mode)
                    self.bridge.publish_mode(mvec)
                    self._log(
                        f'{self.joint_names[idx]}: CSP mode injected safely')

                QTimer.singleShot(200, _inject)

            QTimer.singleShot(50, _publish_and_inject)
        else:
            # 非 CSP 模式：直接注入 mode，延迟切 controller
            mvec = [0.0] * self.num
            for i, p in enumerate(self.panels):
                mvec[i] = float(p.mode_display or MODE.NO_MODE)
            mvec[idx] = float(target_mode)
            self.bridge.publish_mode(mvec)

            def do_switch():
                self._switch_motion_controller(target_ctrl)

            QTimer.singleShot(200, do_switch)

    # ---- PP one-shot target ----
    def _on_send_pp(self, idx: int):
        if self._active_ctrl != 'pp_controller':
            self._log('Active controller is not pp_controller; '
                      'click "Apply" after selecting PP first.', 'warn')
            return

        target = self.panels[idx].position_setpoint_deg()
        vec = self._feedback_position_vector()
        vec[idx] = float(target)
        self.bridge.publish_pp(vec)

        # CiA 402 Profile Position starts on a rising edge of Control Word
        # bit 4. Keep bit 5 set so the drive applies the new target now.
        cw_start = self._current_cw_vector()
        cw_start[idx] = float(
            CW.ENABLE_OPERATION | CW.NEW_SET_POINT | CW.CHANGE_IMMEDIATE)
        self.bridge.publish_control_word(cw_start)
        self._log(
            f'{self.joint_names[idx]}: PP target '
            f'{self._format_panel_value(idx, target)}')

        def _clear_new_set_point():
            cw_hold = self._current_cw_vector()
            cw_hold[idx] = float(CW.ENABLE_OPERATION)
            self.bridge.publish_control_word(cw_hold)

        QTimer.singleShot(100, _clear_new_set_point)

    # ---- CSP streaming ----
    def _on_stream_start(self, idx: int):
        if self._active_ctrl != 'csp_controller':
            self._log('Active controller is not csp_controller; '
                      'click "Apply" after selecting CSP first.', 'warn')
            return

        # 指令初始化对齐（保险措施，防止用户在 Apply 之后又拖动滑条）：
        # 在第一帧出去之前，把 spinbox/slider 强制对齐到当前反馈位置。
        # 由于 _tick_csp_stream 的下一拍 (10 ms 后) 才会取 spn 值，这里
        # 提前 setValue 即可保证首帧 setpoint == 当前 feedback。
        cur = self.panels[idx].align_setpoint_to_feedback()
        self._log(f'{self.joint_names[idx]}: CSP stream start, '
                  f'first setpoint aligned to '
                  f'{self._format_panel_value(idx, cur)}')

        self._streaming_csp[idx] = True
        self.panels[idx].btn_stream_start.setEnabled(False)
        self.panels[idx].btn_stream_stop.setEnabled(True)
        if not self._csp_timer.isActive():
            self._csp_timer.start()

    def _on_stream_stop(self, idx: int):
        self._streaming_csp[idx] = False
        self.panels[idx].btn_stream_start.setEnabled(True)
        self.panels[idx].btn_stream_stop.setEnabled(False)
        if not any(self._streaming_csp):
            self._csp_timer.stop()

    def _tick_csp_stream(self):
        """
        Called 100 Hz while any joint is streaming. Publishes
        JointGroupPositionController expects size==num_joints, so
        we always fill the whole vector - joints not streaming
        keep their last measured position (no movement).

        目标位置以 deg 写入 unit_converter，再由其转换到底层 counts。
        """
        vec = self._feedback_position_vector()
        for i, streaming in enumerate(self._streaming_csp):
            if streaming:
                vec[i] = float(self.panels[i].position_setpoint_deg())
        self.bridge.publish_csp(vec)

    # ---- Velocity one-shot ----
    def _on_send_vel(self, idx: int):
        v = self.panels[idx].velocity_setpoint_deg_s()
        if self._active_ctrl == 'csv_controller':
            self._csv_targets[idx] = float(v)
            self.bridge.publish_csv(self._csv_targets)
            self._log(
                f'{self.joint_names[idx]}: CSV '
                f'{v:+.3f} {self._panel_unit(idx)}/s')
        elif self._active_ctrl == 'pv_controller':
            self._pv_targets[idx] = float(v)
            self.bridge.publish_pv(self._pv_targets)
            self._log(
                f'{self.joint_names[idx]}: PV '
                f'{v:+.3f} {self._panel_unit(idx)}/s')
        else:
            self._log('Select CSV or PV mode and press Apply first.', 'warn')

    # ------------------------------------------------------
    def _current_cw_vector(self) -> List[float]:
        """
        Keep Control Word values of other joints at their current
        state (Disable=0 is a safe default for any motor we are not
        touching - but we'd ideally keep Enable for already-enabled
        ones). Here we re-compute each joint's correct CW from its
        status word so we never accidentally demote another joint.
        """
        vec: List[float] = []
        for p in self.panels:
            st = parse_status_word(p.status_word)
            if st.operation_enabled:
                vec.append(float(CW.ENABLE_OPERATION))
            elif st.switched_on:
                vec.append(float(CW.SWITCH_ON))
            elif st.ready_to_switch_on:
                vec.append(float(CW.SHUTDOWN))
            else:
                vec.append(float(CW.DISABLE_VOLTAGE))
        return vec

    # ------------------------------------------------------
    def _log(self, msg: str, level: str = 'info'):
        color = {'info': 'black', 'warn': 'orange', 'error': 'red'}.get(level, 'black')
        self.txt_log.append(f'<span style="color:{color};">[{level}] {msg}</span>')

    # ------------------------------------------------------
    def shutdown(self):
        self._csp_timer.stop()
        self._calib_timer.stop()
        self._cancel_enable_all_timers()
        for t in self._enable_timers.values():
            t.stop()
        # Ensure drives go back to disabled on exit.
        try:
            self.bridge.publish_control_word([0.0] * self.num)
        except Exception:                            # noqa: BLE001
            pass
        self.bridge.stop()


# ==========================================================
# rqt Plugin shell
# ==========================================================
def _resolve_joint_names(timeout: float = 3.0) -> List[str]:
    """
    Resolve GUI joint names for both rqt and standalone entry-points.
    Priority:
      1. JOINT_NAMES env var, comma-separated
      2. First /dynamic_joint_states message
      3. MultiMotorWidget.DEFAULT_JOINTS
    """
    env_names = os.environ.get('JOINT_NAMES', '').strip()
    if env_names:
        joints = [j.strip() for j in env_names.split(',') if j.strip()]
        if joints:
            return joints

    joints = MotorRQTPlugin._discover_joints_from_ros(timeout=timeout)
    return joints if joints else MultiMotorWidget.DEFAULT_JOINTS


class MotorRQTPlugin(Plugin):
    """rqt entry-point discovered through plugin.xml."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('MotorRQTPlugin')

        # --- Auto-discover joint names -------------------------------------------
        # Priority (highest first):
        #   1. JOINT_NAMES env var  (comma-separated, e.g. "joint_1,joint_3,joint_4")
        #   2. /dynamic_joint_states  (first message, wait up to 3 s)
        #   3. Hard-coded default: joint_1 .. joint_5
        #
        # This ensures the plugin publishes the correct array length when the bus
        # only has a subset of joints (e.g. joint_1+joint_3+joint_4).
        # -------------------------------------------------------------------------
        self._widget = MultiMotorWidget(joint_names=_resolve_joint_names())
        self._widget.setObjectName('MultiMotorWidget')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})')
        context.add_widget(self._widget)

    @staticmethod
    def _discover_joints_from_ros(timeout: float = 3.0) -> List[str] | None:
        """
        Subscribe to /dynamic_joint_states for `timeout` seconds and return
        the sorted list of joint names found in the first message.
        Returns None if no message arrives in time.
        """
        found: List[str] = []
        event = threading.Event()

        if not rclpy.ok():
            rclpy.init(args=None)

        node = rclpy.create_node('_motor_plugin_discovery')

        def _cb(msg: DynamicJointState):
            if msg.joint_names:
                found.extend(sorted(msg.joint_names))
                event.set()

        sub = node.create_subscription(
            DynamicJointState, '/dynamic_joint_states', _cb, 1)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.time() + timeout
        while not event.is_set() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_subscription(sub)
        node.destroy_node()

        return found if found else None

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


# ==========================================================
# Standalone entry-point (for quick testing without rqt)
#   ros2 run multi_motor_control my_motor_rqt_plugin
# ==========================================================
def main():
    import sys
    from python_qt_binding.QtWidgets import QApplication
    app = QApplication(sys.argv)
    w = MultiMotorWidget(joint_names=_resolve_joint_names())
    w.resize(1000, 800)
    w.setWindowTitle('Denali XCR Multi-Motor (standalone)')
    w.show()
    try:
        rc = app.exec_()
    finally:
        w.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
