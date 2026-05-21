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
  /multi_motor/states_deg                (sub)  position_deg & velocity_deg_s
  /dynamic_joint_states                  (sub)  status_word, mode disp
  /cia402_cmd_controller/commands        (pub)  Control Word array
  /cia402_mode_controller/commands       (pub)  Mode array
  /fault_reset_controller/commands       (pub)  reset_fault pulse
  /multi_motor/pp_position_deg/commands       (pub)  profile-position target
  /multi_motor/csp_position_deg/commands      (pub)  position setpoint
  /multi_motor/csv_velocity_deg_s/commands    (pub)  velocity setpoint
  /multi_motor/pv_velocity_deg_s/commands     (pub)  target velocity in PV mode
  /controller_manager/list_controllers   (srv)
  /controller_manager/switch_controller  (srv)
"""

from __future__ import annotations

import os
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
except ImportError:
    # Running this file directly, e.g. `python3 my_motor_rqt_plugin.py`
    from cia402 import CW, MODE, parse_status_word, next_control_word


# ==========================================================
# ROS <-> Qt bridge
# ==========================================================
class RosBridge(QObject):
    """
    Owns an rclpy Node + a private executor in a background thread.
    Exposes Qt signals for UI thread consumption.

    Public Qt signals
    -----------------
      joint_state_updated(list[float] pos_deg, list[float] vel_deg_s)
      dynamic_state_updated(list[int] status_words, list[int] mode_disp)
      controllers_updated(dict[str, str] name -> state)
      log_message(str level, str msg)
    """

    joint_state_updated   = pyqtSignal(list, list)
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
        self._pub_csv  = n.create_publisher(
            Float64MultiArray, '/multi_motor/csv_velocity_deg_s/commands', 10)
        self._pub_pv   = n.create_publisher(
            Float64MultiArray, '/multi_motor/pv_velocity_deg_s/commands', 10)

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
        for jn, ifv in zip(msg.joint_names, msg.interface_values):
            if jn not in self._joint_names:
                continue
            i = self._joint_names.index(jn)
            for name, val in zip(ifv.interface_names, ifv.values):
                if name == 'position_deg':
                    pos[i] = val
                elif name == 'velocity_deg_s':
                    vel[i] = val
        self.joint_state_updated.emit(pos, vel)

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

    def publish_csv(self, vec: List[float]):
        self._publish_array(self._pub_csv, vec)

    def publish_pv(self, vec: List[float]):
        self._publish_array(self._pub_pv, vec)

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

    # Slider/Spinbox 允许的位置范围，以输出轴转数为单位。
    # 默认 ±5 圈，足够常规调试且不会一不小心把电机撞飞。
    POS_RANGE_REVS = 5
    # 速度目标范围：±5 圈/秒，可按需放宽。
    VEL_RANGE_REVS_PER_S = 5
    POS_SLIDER_SCALE = 10  # slider integer step = 0.1 deg

    def __init__(self, joint_name: str, index: int,
                 encoder_resolution: float, parent=None):
        super().__init__(joint_name, parent)
        self.joint_name = joint_name
        self.index = index
        self.encoder_resolution = encoder_resolution

        self._cached_status_word = 0
        self._cached_mode_disp   = 0
        self._cached_pos_deg     = 0.0
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

        s.addWidget(QLabel('Position:'), row, 0)
        self.lbl_pos = QLabel('0.000 deg')
        s.addWidget(self.lbl_pos, row, 1)
        row += 1

        s.addWidget(QLabel('Velocity:'), row, 0)
        self.lbl_vel = QLabel('0.000 deg/s')
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
        mode_row.addWidget(self.cmb_mode, 1)
        self.btn_apply_mode = QPushButton('Apply')
        mode_row.addWidget(self.btn_apply_mode)
        root.addLayout(mode_row)

        # ---- CSP controls ----------------------------
        # 位置目标单位：角度。ROS 层 unit_converter 负责 deg -> counts。
        pos_limit_deg = self.POS_RANGE_REVS * 360.0
        pos_slider_limit = int(pos_limit_deg * self.POS_SLIDER_SCALE)

        gb_csp = QGroupBox('PP / CSP - Position setpoint (deg)')
        csp = QVBoxLayout(gb_csp)
        csp_row = QHBoxLayout()
        self.sld_pos = QSlider(Qt.Horizontal)
        self.sld_pos.setRange(-pos_slider_limit, pos_slider_limit)
        self.sld_pos.setSingleStep(1)
        self.sld_pos.setPageStep(10)
        self.sld_pos.setValue(0)
        csp_row.addWidget(self.sld_pos, 1)
        self.spn_pos = QDoubleSpinBox()
        self.spn_pos.setDecimals(3)
        self.spn_pos.setRange(-pos_limit_deg, pos_limit_deg)
        self.spn_pos.setSingleStep(1.0)
        self.spn_pos.setSuffix(' deg')
        self.spn_pos.setGroupSeparatorShown(True)
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
        root.addWidget(gb_csp)

        # ---- CSV / PV controls -----------------------
        # 速度目标单位：deg/s。ROS 层 unit_converter 负责 deg/s -> counts/s。
        vel_limit_deg_s = self.VEL_RANGE_REVS_PER_S * 360.0

        gb_vel = QGroupBox('CSV / PV - Velocity setpoint (deg/s)')
        vel = QHBoxLayout(gb_vel)
        self.spn_vel = QDoubleSpinBox()
        self.spn_vel.setDecimals(3)
        self.spn_vel.setRange(-vel_limit_deg_s, vel_limit_deg_s)
        self.spn_vel.setSingleStep(10.0)
        self.spn_vel.setSuffix(' deg/s')
        self.spn_vel.setGroupSeparatorShown(True)
        vel.addWidget(self.spn_vel, 1)
        self.btn_send_vel = QPushButton('Send (one-shot)')
        self.btn_send_vel.setStyleSheet('background-color:#9C27B0; color:white;')
        vel.addWidget(self.btn_send_vel)
        root.addWidget(gb_vel)

        # ---- slider/spin linkage ---------------------
        self.sld_pos.valueChanged.connect(
            lambda v: self.spn_pos.setValue(v / self.POS_SLIDER_SCALE))
        self.spn_pos.valueChanged.connect(
            lambda v: self.sld_pos.setValue(
                int(round(v * self.POS_SLIDER_SCALE))))

    # ------------------------------------------------------
    # public getters
    # ------------------------------------------------------
    def selected_mode(self) -> int:
        return self.MODES_UI[self.cmb_mode.currentIndex()][1]

    def position_setpoint_deg(self) -> float:
        """PP/CSP 目标位置，单位：deg。"""
        return float(self.spn_pos.value())

    def velocity_setpoint_deg_s(self) -> float:
        """CSV/PV 目标速度，单位：deg/s。"""
        return float(self.spn_vel.value())

    # ------------------------------------------------------
    # alignment helpers
    # ------------------------------------------------------
    def align_setpoint_to_feedback(self) -> float:
        """
        指令初始化对齐：把 PP/CSP 的位置 spinbox/slider 强制设置为
        当前反馈位置 (`_cached_pos_deg`)，并把 spin/slider 的
        允许范围动态调整为「当前位置 ± POS_RANGE_REVS 圈」。

        必须在以下两个时机调用：
          1. 切到 CSP 模式（Apply）后：让用户看到的滑条停在「当前点」，
             避免后续误操作把电机拽回零位。
          2. 开始 CSP streaming 前：保证 _tick_csp_stream 发出的
             第一帧位置指令 = 当前反馈位置，避免电机阶跃。

        返回对齐后的目标值（deg）。
        """
        cur = float(self._cached_pos_deg)
        span = self.POS_RANGE_REVS * 360.0
        new_min = cur - span
        new_max = cur + span
        new_min_slider = int(round(new_min * self.POS_SLIDER_SCALE))
        new_max_slider = int(round(new_max * self.POS_SLIDER_SCALE))
        # 阻止 valueChanged 在 setRange/setValue 期间产生抖动信号
        self.sld_pos.blockSignals(True)
        self.spn_pos.blockSignals(True)
        try:
            self.sld_pos.setRange(new_min_slider, new_max_slider)
            self.spn_pos.setRange(new_min, new_max)
            self.sld_pos.setValue(int(round(cur * self.POS_SLIDER_SCALE)))
            self.spn_pos.setValue(cur)
        finally:
            self.sld_pos.blockSignals(False)
            self.spn_pos.blockSignals(False)
        return cur

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

    def update_joint_state(self, pos_deg: float, vel_deg_s: float):
        """
        /multi_motor/states_deg 由 unit_converter 从底层 counts 换算而来。
        GUI 只显示和下发角度单位。
        """
        self._cached_pos_deg = float(pos_deg)
        self._cached_vel_deg_s = float(vel_deg_s)
        revs = pos_deg / 360.0
        rps = vel_deg_s / 360.0
        self.lbl_pos.setText(f'{pos_deg:+.3f} deg   ({revs:+.3f} rev)')
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
    DEFAULT_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    DEFAULT_ENCODER_RES = 865075.2   # 2^17 * 6.6 gearbox

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

    # Staggered enable: how many joints in one batch, and the delay between
    # batches (ms). Used by _start_enable_all_staggered() so a shared / weak
    # 24 V supply does not see all motors' inrush at the exact same moment
    # (which on this rig drops the bus below 22 V and trips drives 3+4 with
    # CiA 402 error 0x3280 "DC link voltage").
    STAGGER_BATCH_SIZE = 1     # enable 1 joint at a time
    STAGGER_DELAY_MS   = 600   # gap between batches

    def __init__(self,
                 joint_names: Optional[List[str]] = None,
                 encoder_resolution: float = DEFAULT_ENCODER_RES,
                 parent=None):
        super().__init__(parent)
        self.joint_names = joint_names or self.DEFAULT_JOINTS
        self.num = len(self.joint_names)
        self.encoder_resolution = encoder_resolution

        # State
        self._active_ctrl: Optional[str] = None
        self._streaming_csp = [False] * self.num
        self._csv_targets = [0.0] * self.num
        self._pv_targets = [0.0] * self.num

        # ROS bridge
        self.bridge = RosBridge(self.joint_names)
        self.bridge.joint_state_updated.connect(self._on_joint_state)
        self.bridge.dynamic_state_updated.connect(self._on_dynamic_state)
        self.bridge.controllers_updated.connect(self._on_controllers)
        self.bridge.log_message.connect(self._on_log)

        # UI
        self._build_ui()

        # CSP streaming timer
        self._csp_timer = QTimer(self)
        self._csp_timer.setInterval(int(1000 / self.CSP_STREAM_HZ))
        self._csp_timer.timeout.connect(self._tick_csp_stream)

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

        # Global toolbar (staggered enable / disable all)
        bar = QHBoxLayout()
        self.btn_enable_all = QPushButton(
            f'Enable All  (staggered: {self.STAGGER_BATCH_SIZE} per '
            f'{self.STAGGER_DELAY_MS} ms)')
        self.btn_enable_all.setStyleSheet(
            'background:#2E7D32; color:white; font-weight:bold; padding:6px;')
        self.btn_disable_all = QPushButton('Disable All')
        self.btn_disable_all.setStyleSheet(
            'background:#C62828; color:white; font-weight:bold; padding:6px;')
        bar.addWidget(self.btn_enable_all)
        bar.addWidget(self.btn_disable_all)
        bar.addStretch(1)
        root.addLayout(bar)
        self.btn_enable_all.clicked.connect(self._start_enable_all_staggered)
        self.btn_disable_all.clicked.connect(self._disable_all)

        # Panel grid (scrollable so 3->8 motors still fits)
        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        self.panels: List[MotorPanel] = []
        for i, jn in enumerate(self.joint_names):
            p = MotorPanel(jn, i, self.encoder_resolution)
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
    @pyqtSlot(list, list)
    def _on_joint_state(self, pos, vel):
        for i, p in enumerate(self.panels):
            p.update_joint_state(pos[i], vel[i])

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

        # CSP 使用 CSV 引导；其他模式直接使能
        enable_mode = MODE.CSV if user_mode == MODE.CSP else user_mode

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
                f'aligned to feedback @ {cur:+.3f} deg')

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
        to_deact = [c for c in self.ALL_MOTION_CTRLS
                    if c != target_ctrl and c == self._active_ctrl]
        self.bridge.switch_controller(
            activate=[target_ctrl],
            deactivate=to_deact,
            strictness=SwitchController.Request.BEST_EFFORT,
        )
        self._active_ctrl = target_ctrl
        self._log(
            f'{self.joint_names[idx]}: activated {target_ctrl}')

        # ---- Step 2: CSP 模式特殊处理 ----
        if target_mode == MODE.CSP:
            # 发送当前反馈位置，覆盖 command_interface 中可能的陈旧值
            cur = self.panels[idx].align_setpoint_to_feedback()
            cur_pos_vec = [float(p._cached_pos_deg) for p in self.panels]

            def _publish_and_switch():
                # 连续发送位置确保 controller 收到
                self.bridge.publish_csp(cur_pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: CSP position = '
                    f'{cur:+.3f} deg, waiting for CI to stabilize...')

                def _inject_csp_mode():
                    # 再发一次位置确保万无一失
                    fresh_pos = [float(p._cached_pos_deg)
                                 for p in self.panels]
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
        self._log(f'{self.joint_names[idx]}: disable')
        t = self._enable_timers.pop(idx, None)
        if t is not None:
            t.stop()
        vec = self._current_cw_vector()
        vec[idx] = float(CW.DISABLE_VOLTAGE)
        self.bridge.publish_control_word(vec)

    # ------------------------------------------------------
    # Staggered (batched) enable / disable for all joints
    # ------------------------------------------------------
    def _start_enable_all_staggered(self):
        """
        Enable all joints in batches of STAGGER_BATCH_SIZE separated by
        STAGGER_DELAY_MS. This prevents simultaneous inrush on a shared
        24 V supply, which on this rig was sagging the DC link below the
        drive's UV threshold (CiA 402 error 0x3280) and tripping the
        last motors in the daisy-chain (typically joint_3 / joint_4).

        Joints that are already in fault are skipped here -- the user
        should hit "Reset Fault" first; otherwise we only fight ourselves.
        """
        batch_sz = max(1, int(self.STAGGER_BATCH_SIZE))
        delay_ms = max(0, int(self.STAGGER_DELAY_MS))

        targets: list[int] = []
        for i, p in enumerate(self.panels):
            st = parse_status_word(p.status_word)
            if st.fault:
                self._log(
                    f'{self.joint_names[i]}: skipped by Enable All '
                    f'(in FAULT, reset first)', 'warn')
                continue
            if st.operation_enabled:
                continue   # already on, nothing to do
            targets.append(i)

        if not targets:
            self._log('Enable All: nothing to do (all enabled or faulted)')
            return

        self._log(
            f'Enable All (staggered): {len(targets)} joint(s), '
            f'{batch_sz} per batch, {delay_ms} ms apart -> '
            f'{[self.joint_names[i] for i in targets]}')

        def kick_batch(start: int):
            batch = targets[start:start + batch_sz]
            for idx in batch:
                self._log(
                    f'  staggered enable -> {self.joint_names[idx]}')
                self._start_enable_sequence(idx)
            nxt = start + batch_sz
            if nxt < len(targets):
                QTimer.singleShot(delay_ms, lambda: kick_batch(nxt))

        kick_batch(0)

    def _disable_all(self):
        """Disable every joint at once (safe: just CW=0x0000)."""
        self._log('Disable All')
        for t in list(self._enable_timers.values()):
            t.stop()
        self._enable_timers.clear()
        self.bridge.publish_control_word(
            [float(CW.DISABLE_VOLTAGE)] * self.num)

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
                      f'aligned to feedback {cur:+.3f} deg')

        if target_mode == MODE.CSP:
            # CSP 安全切换时序：
            # 1. 先激活 csp_controller（还在旧模式，override_command=true，安全）
            # 2. 发送当前位置到 controller
            # 3. 等 command_interface 稳定
            # 4. 最后再注入 CSP 模式
            cur = self.panels[idx].align_setpoint_to_feedback()
            self._log(f'{self.joint_names[idx]}: CSP setpoint '
                      f'aligned to feedback {cur:+.3f} deg')

            to_deact = [c for c in self.ALL_MOTION_CTRLS
                        if c != target_ctrl and c == self._active_ctrl]
            self.bridge.switch_controller(
                activate=[target_ctrl],
                deactivate=to_deact,
                strictness=SwitchController.Request.BEST_EFFORT,
            )
            self._active_ctrl = target_ctrl
            self._log(f'{self.joint_names[idx]}: activated {target_ctrl}')

            def _publish_and_inject():
                pos_vec = [float(p._cached_pos_deg) for p in self.panels]
                self.bridge.publish_csp(pos_vec)
                self._log(
                    f'{self.joint_names[idx]}: published CSP position, '
                    f'waiting for CI to stabilize...')

                def _inject():
                    fresh = [float(p._cached_pos_deg)
                             for p in self.panels]
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
                to_deact = [c for c in self.ALL_MOTION_CTRLS
                            if c != target_ctrl and c == self._active_ctrl]
                self.bridge.switch_controller(
                    activate=[target_ctrl],
                    deactivate=to_deact,
                    strictness=SwitchController.Request.BEST_EFFORT,
                )
                self._active_ctrl = target_ctrl

            QTimer.singleShot(200, do_switch)

    # ---- PP one-shot target ----
    def _on_send_pp(self, idx: int):
        if self._active_ctrl != 'pp_controller':
            self._log('Active controller is not pp_controller; '
                      'click "Apply" after selecting PP first.', 'warn')
            return

        target = self.panels[idx].position_setpoint_deg()
        vec = [float(p._cached_pos_deg) for p in self.panels]
        vec[idx] = float(target)
        self.bridge.publish_pp(vec)

        # CiA 402 Profile Position starts on a rising edge of Control Word
        # bit 4. Keep bit 5 set so the drive applies the new target now.
        cw_start = self._current_cw_vector()
        cw_start[idx] = float(
            CW.ENABLE_OPERATION | CW.NEW_SET_POINT | CW.CHANGE_IMMEDIATE)
        self.bridge.publish_control_word(cw_start)
        self._log(f'{self.joint_names[idx]}: PP target {target:+.3f} deg')

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
                  f'first setpoint aligned to {cur:+.3f} deg')

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
        vec = [float(p._cached_pos_deg) for p in self.panels]
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
            self._log(f'{self.joint_names[idx]}: CSV {v:+.3f} deg/s')
        elif self._active_ctrl == 'pv_controller':
            self._pv_targets[idx] = float(v)
            self.bridge.publish_pv(self._pv_targets)
            self._log(f'{self.joint_names[idx]}: PV {v:+.3f} deg/s')
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
        #   3. Hard-coded default: joint_1 .. joint_4
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
