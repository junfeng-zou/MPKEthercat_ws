#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-level unit converter for the multi-motor CiA 402 stack.

The EtherCAT/ros2_control side intentionally stays in drive-native
counts/counts/s. This node exposes degree-based command topics for
higher-level software and republishes feedback in degree units.
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from multi_motor_control.joint_limits import load_joint_limits
except ImportError:
    from joint_limits import load_joint_limits


INT32_MIN = -2147483648
INT32_MAX = 2147483647
MOTOR_ENCODER_RESOLUTION = 131072.0  # 2^17 counts/motor rev
GEAR_RATIO = 6.6
DEFAULT_ENCODER_RESOLUTION = MOTOR_ENCODER_RESOLUTION * GEAR_RATIO
DEFAULT_ENCODER_STATE_SAVE_PERIOD_SEC = 0.02
DEFAULT_ENCODER_STATE_MIN_DELTA_COUNTS = 1
DEFAULT_ENCODER_REALIGN_THRESHOLD_COUNTS = int(
    MOTOR_ENCODER_RESOLUTION / 2.0)
DEFAULT_ENCODER_STATUS_TIMEOUT_SEC = 0.05
DEFAULT_ENCODER_REALIGN_SAVE_HOLDOFF_SEC = 0.5
DEFAULT_ENCODER_POWERUP_STABILIZE_SEC = 0.2
DEFAULT_ENCODER_RAW_STABLE_DURATION_SEC = 0.1
DEFAULT_ENCODER_RAW_STABLE_DELTA_COUNTS = 128


def default_encoder_state_file() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    for parent in package_root.parents:
        if (parent / 'src' / 'multi_motor').exists():
            return parent / 'state' / 'multi_motor_encoder_state.json'
    return package_root / 'state' / 'multi_motor_encoder_state.json'


DEFAULT_ENCODER_STATE_FILE = default_encoder_state_file()


class EncoderTurnState:
    """Persist and restore one joint's continuous encoder count."""

    def __init__(
            self,
            joint_name: str,
            one_turn_count: int,
            saved: Optional[dict] = None) -> None:
        self.joint_name = joint_name
        self.one_turn_count = int(one_turn_count)
        self.raw_6064_cnt = 0
        self.restore_offset_cnt = 0
        self.continuous_cnt = 0
        self.turn = 0
        self.single_cnt = 0
        self.valid = False
        self.restored_from_saved = False
        self.poweroff_delta_cnt = 0
        self.restore_error_at_boot_cnt = 0
        self.realign_count = 0
        self.last_realign_delta_cnt = 0
        self.last_realign_raw_before_cnt = 0
        self.last_realign_raw_after_cnt = 0
        self.last_realign_raw_delta_cnt = 0
        self.last_realign_raw_delta_turns = 0
        self.last_realign_raw_delta_residual_cnt = 0
        self._saved_continuous_cnt: Optional[int] = None
        self._initialized = False
        self._last_saved_continuous_cnt: Optional[int] = None

        if saved and saved.get('valid', False):
            try:
                if int(saved.get('one_turn_cnt', one_turn_count)) == one_turn_count:
                    self._saved_continuous_cnt = int(round(
                        float(saved['continuous_cnt'])))
                    self.realign_count = int(saved.get('realign_count', 0))
                    self.last_realign_delta_cnt = int(
                        saved.get('last_realign_delta_cnt', 0))
                    self.last_realign_raw_before_cnt = int(
                        saved.get('last_realign_raw_before_cnt', 0))
                    self.last_realign_raw_after_cnt = int(
                        saved.get('last_realign_raw_after_cnt', 0))
                    self.last_realign_raw_delta_cnt = int(
                        saved.get('last_realign_raw_delta_cnt', 0))
                    self.last_realign_raw_delta_turns = int(
                        saved.get('last_realign_raw_delta_turns', 0))
                    self.last_realign_raw_delta_residual_cnt = int(
                        saved.get('last_realign_raw_delta_residual_cnt', 0))
            except (KeyError, TypeError, ValueError):
                self._saved_continuous_cnt = None

    @staticmethod
    def _nearest_turn_offset(delta: int, one_turn_count: int) -> int:
        half_turn = one_turn_count // 2
        if delta >= 0:
            return (delta + half_turn) // one_turn_count
        return -((-delta + half_turn) // one_turn_count)

    def update(
            self,
            raw_6064_cnt: int,
            realign_threshold_counts: int,
            allow_realign: bool = True) -> Optional[dict]:
        """Update from the current drive count.

        Returns realignment details when a large raw-count jump is detected.
        """
        realign_info = None
        if not self._initialized:
            if self._saved_continuous_cnt is not None:
                turn_offset = self._nearest_turn_offset(
                    self._saved_continuous_cnt - raw_6064_cnt,
                    self.one_turn_count)
                self.restore_offset_cnt = turn_offset * self.one_turn_count
                self.restored_from_saved = True
                self.restore_error_at_boot_cnt = (
                    raw_6064_cnt + self.restore_offset_cnt -
                    self._saved_continuous_cnt)
            else:
                self.restore_offset_cnt = 0
                self.restored_from_saved = False
            self._initialized = True
        elif self.valid and allow_realign:
            raw_delta = raw_6064_cnt - self.raw_6064_cnt
            if abs(raw_delta) > realign_threshold_counts:
                raw_delta_turns = self._nearest_turn_offset(
                    raw_delta, self.one_turn_count)
                raw_delta_residual = (
                    raw_delta - raw_delta_turns * self.one_turn_count)
                reference_cnt = self.continuous_cnt
                turn_offset = self._nearest_turn_offset(
                    reference_cnt - raw_6064_cnt,
                    self.one_turn_count)
                self.restore_offset_cnt = turn_offset * self.one_turn_count
                realigned_cnt = raw_6064_cnt + self.restore_offset_cnt
                self.last_realign_delta_cnt = realigned_cnt - reference_cnt
                self.last_realign_raw_before_cnt = self.raw_6064_cnt
                self.last_realign_raw_after_cnt = raw_6064_cnt
                self.last_realign_raw_delta_cnt = raw_delta
                self.last_realign_raw_delta_turns = raw_delta_turns
                self.last_realign_raw_delta_residual_cnt = raw_delta_residual
                self.realign_count += 1
                realign_info = {
                    'raw_delta_cnt': raw_delta,
                    'raw_before_cnt': self.last_realign_raw_before_cnt,
                    'raw_after_cnt': raw_6064_cnt,
                    'raw_delta_turns': raw_delta_turns,
                    'raw_delta_residual_cnt': raw_delta_residual,
                    'reference_cnt': reference_cnt,
                    'realigned_cnt': realigned_cnt,
                    'restore_offset_cnt': self.restore_offset_cnt,
                    'realign_delta_cnt': self.last_realign_delta_cnt,
                    'realign_count': self.realign_count,
                }

        self.raw_6064_cnt = raw_6064_cnt
        self.continuous_cnt = raw_6064_cnt + self.restore_offset_cnt
        self.turn = self.continuous_cnt // self.one_turn_count
        self.single_cnt = self.continuous_cnt - (
            self.turn * self.one_turn_count)
        self.valid = True
        if self._saved_continuous_cnt is not None:
            self.poweroff_delta_cnt = (
                self.continuous_cnt - self._saved_continuous_cnt)

        return realign_info

    def needs_save(self, min_delta_counts: int) -> bool:
        if not self.valid:
            return False
        if self._last_saved_continuous_cnt is None:
            return True
        return (
            abs(self.continuous_cnt - self._last_saved_continuous_cnt) >=
            min_delta_counts)

    def mark_saved(self) -> None:
        self._last_saved_continuous_cnt = self.continuous_cnt

    def reset_restore_to_raw(self, raw_6064_cnt: int) -> None:
        """Make the restored continuous count equal the current raw count."""
        raw_6064_cnt = int(raw_6064_cnt)
        self.raw_6064_cnt = raw_6064_cnt
        self.restore_offset_cnt = 0
        self.continuous_cnt = raw_6064_cnt
        self.turn = self.continuous_cnt // self.one_turn_count
        self.single_cnt = self.continuous_cnt - (
            self.turn * self.one_turn_count)
        self.valid = True
        self.restored_from_saved = False
        self.poweroff_delta_cnt = 0
        self.restore_error_at_boot_cnt = 0
        self.last_realign_delta_cnt = 0
        self.last_realign_raw_before_cnt = raw_6064_cnt
        self.last_realign_raw_after_cnt = raw_6064_cnt
        self.last_realign_raw_delta_cnt = 0
        self.last_realign_raw_delta_turns = 0
        self.last_realign_raw_delta_residual_cnt = 0
        self._saved_continuous_cnt = raw_6064_cnt
        self._initialized = True
        self._last_saved_continuous_cnt = None

    def as_dict(self) -> dict:
        return {
            'raw_6064_cnt': self.raw_6064_cnt,
            'restore_offset_cnt': self.restore_offset_cnt,
            'continuous_cnt': self.continuous_cnt,
            'turn': self.turn,
            'single_cnt': self.single_cnt,
            'one_turn_cnt': self.one_turn_count,
            'poweroff_delta_cnt': self.poweroff_delta_cnt,
            'restore_error_at_boot_cnt': self.restore_error_at_boot_cnt,
            'realign_count': self.realign_count,
            'last_realign_delta_cnt': self.last_realign_delta_cnt,
            'last_realign_raw_before_cnt': self.last_realign_raw_before_cnt,
            'last_realign_raw_after_cnt': self.last_realign_raw_after_cnt,
            'last_realign_raw_delta_cnt': self.last_realign_raw_delta_cnt,
            'last_realign_raw_delta_turns':
                self.last_realign_raw_delta_turns,
            'last_realign_raw_delta_residual_cnt':
                self.last_realign_raw_delta_residual_cnt,
            'restored_from_saved': self.restored_from_saved,
            'valid': self.valid,
        }


class UnitConverter(Node):
    """Bridge public degree topics to the lower-level count topics."""

    def __init__(self) -> None:
        super().__init__('multi_motor_unit_converter')

        self.declare_parameter('num_joints', 5)
        self.declare_parameter('encoder_resolution', DEFAULT_ENCODER_RESOLUTION)
        self.declare_parameter('position_limits_file', '')
        self.declare_parameter('joint_geometry_file', '')
        self.declare_parameter(
            'encoder_state_file', str(DEFAULT_ENCODER_STATE_FILE))
        self.declare_parameter(
            'encoder_one_turn_cnt', int(MOTOR_ENCODER_RESOLUTION))
        self.declare_parameter(
            'encoder_state_save_period_sec',
            DEFAULT_ENCODER_STATE_SAVE_PERIOD_SEC)
        self.declare_parameter(
            'encoder_state_min_delta_counts',
            DEFAULT_ENCODER_STATE_MIN_DELTA_COUNTS)
        self.declare_parameter(
            'encoder_realign_threshold_counts',
            DEFAULT_ENCODER_REALIGN_THRESHOLD_COUNTS)
        self.declare_parameter('encoder_require_status_word', True)
        self.declare_parameter(
            'encoder_status_timeout_sec',
            DEFAULT_ENCODER_STATUS_TIMEOUT_SEC)
        self.declare_parameter(
            'encoder_realign_save_holdoff_sec',
            DEFAULT_ENCODER_REALIGN_SAVE_HOLDOFF_SEC)
        self.declare_parameter(
            'encoder_powerup_stabilize_sec',
            DEFAULT_ENCODER_POWERUP_STABILIZE_SEC)
        self.declare_parameter(
            'encoder_raw_stable_duration_sec',
            DEFAULT_ENCODER_RAW_STABLE_DURATION_SEC)
        self.declare_parameter(
            'encoder_raw_stable_delta_counts',
            DEFAULT_ENCODER_RAW_STABLE_DELTA_COUNTS)
        self.declare_parameter('raw_joint_states_topic', '/joint_states_raw')
        self.declare_parameter('restored_joint_states_topic', '/joint_states')

        self._num_joints = int(self.get_parameter('num_joints').value)
        self._encoder_resolution = float(
            self.get_parameter('encoder_resolution').value)
        position_limits_file = str(
            self.get_parameter('position_limits_file').value or '')
        joint_geometry_file = str(
            self.get_parameter('joint_geometry_file').value or '')
        self._encoder_state_file = Path(str(
            self.get_parameter('encoder_state_file').value or
            DEFAULT_ENCODER_STATE_FILE))
        self._encoder_one_turn_cnt = int(
            self.get_parameter('encoder_one_turn_cnt').value)
        self._encoder_state_save_period_sec = float(
            self.get_parameter('encoder_state_save_period_sec').value)
        self._encoder_state_min_delta_counts = int(
            self.get_parameter('encoder_state_min_delta_counts').value)
        self._encoder_realign_threshold_counts = int(
            self.get_parameter('encoder_realign_threshold_counts').value)
        self._encoder_require_status_word = bool(
            self.get_parameter('encoder_require_status_word').value)
        self._encoder_status_timeout_sec = float(
            self.get_parameter('encoder_status_timeout_sec').value)
        self._encoder_realign_save_holdoff_sec = float(
            self.get_parameter('encoder_realign_save_holdoff_sec').value)
        self._encoder_powerup_stabilize_sec = float(
            self.get_parameter('encoder_powerup_stabilize_sec').value)
        self._encoder_raw_stable_duration_sec = float(
            self.get_parameter('encoder_raw_stable_duration_sec').value)
        self._encoder_raw_stable_delta_counts = int(
            self.get_parameter('encoder_raw_stable_delta_counts').value)
        self._raw_joint_states_topic = str(
            self.get_parameter('raw_joint_states_topic').value or
            '/joint_states_raw')
        self._restored_joint_states_topic = str(
            self.get_parameter('restored_joint_states_topic').value or
            '/joint_states')
        self.declare_parameter(
            'joint_names',
            [f'joint_{i}' for i in range(1, self._num_joints + 1)])
        configured_names = list(self.get_parameter('joint_names').value)
        self._joint_names = configured_names
        self._num_joints = len(self._joint_names)

        if self._encoder_resolution <= 0.0:
            raise ValueError('encoder_resolution must be positive')
        if self._encoder_one_turn_cnt <= 0:
            raise ValueError('encoder_one_turn_cnt must be positive')
        if self._encoder_state_save_period_sec < 0.0:
            raise ValueError('encoder_state_save_period_sec cannot be negative')
        if self._encoder_state_min_delta_counts < 0:
            raise ValueError(
                'encoder_state_min_delta_counts cannot be negative')
        if self._encoder_realign_threshold_counts <= 0:
            raise ValueError(
                'encoder_realign_threshold_counts must be positive')
        if self._encoder_status_timeout_sec <= 0.0:
            raise ValueError('encoder_status_timeout_sec must be positive')
        if self._encoder_realign_save_holdoff_sec < 0.0:
            raise ValueError(
                'encoder_realign_save_holdoff_sec cannot be negative')
        if self._encoder_powerup_stabilize_sec < 0.0:
            raise ValueError(
                'encoder_powerup_stabilize_sec cannot be negative')
        if self._encoder_raw_stable_duration_sec < 0.0:
            raise ValueError(
                'encoder_raw_stable_duration_sec cannot be negative')
        if self._encoder_raw_stable_delta_counts < 0:
            raise ValueError(
                'encoder_raw_stable_delta_counts cannot be negative')

        self._counts_per_degree = self._encoder_resolution / 360.0
        self._degree_per_count = 360.0 / self._encoder_resolution
        self._position_limits_file = position_limits_file
        self._joint_geometry_file = joint_geometry_file
        self._position_limits = {}
        self._position_limits_path = None
        self._reload_position_limits()
        self._reported_limit_clamps = set()
        self._reported_position_restore_not_ready = set()
        self.create_timer(1.0, self._reload_position_limits)
        saved_encoder_states = self._load_encoder_state_file()
        self._encoder_states: Dict[str, EncoderTurnState] = {
            joint_name: EncoderTurnState(
                joint_name,
                self._encoder_one_turn_count_for_joint(joint_name),
                saved_encoder_states.get(joint_name))
            for joint_name in self._joint_names
        }
        self._status_words: Dict[str, Optional[int]] = {
            joint_name: None for joint_name in self._joint_names
        }
        self._status_word_valid: Dict[str, bool] = {
            joint_name: not self._encoder_require_status_word
            for joint_name in self._joint_names
        }
        self._status_word_times: Dict[str, float] = {
            joint_name: 0.0 for joint_name in self._joint_names
        }
        self._raw_stabilizing: Dict[str, bool] = {
            joint_name: self._encoder_require_status_word
            for joint_name in self._joint_names
        }
        self._raw_stabilize_not_before: Dict[str, float] = {
            joint_name: 0.0 for joint_name in self._joint_names
        }
        self._raw_stable_since: Dict[str, float] = {
            joint_name: 0.0 for joint_name in self._joint_names
        }
        self._raw_stability_last_cnt: Dict[str, Optional[int]] = {
            joint_name: None for joint_name in self._joint_names
        }
        self._reported_invalid_status = set()
        self._reported_stabilizing = set()
        self._reported_zero_raw_init = set()
        self._reported_save_suppressed = False
        self._reported_zero_raw_save_skip = False
        self._last_encoder_state_save_time = 0.0
        self._encoder_save_holdoff_until = 0.0
        self._encoder_save_suppressed = False
        self._encoder_save_suppression_reason = ''
        self._encoder_realign_armed: Dict[str, bool] = {
            joint_name: self._encoder_require_status_word
            for joint_name in self._joint_names
        }

        self._pub_pp_counts = self.create_publisher(
            Float64MultiArray, '/pp_controller/commands', 10)
        self._pub_csp_counts = self.create_publisher(
            Float64MultiArray, '/csp_controller/commands', 10)
        self._pub_csv_counts = self.create_publisher(
            Float64MultiArray, '/csv_controller/commands', 10)
        self._pub_pv_counts = self.create_publisher(
            Float64MultiArray, '/pv_controller/commands', 10)
        self._pub_state_deg = self.create_publisher(
            DynamicJointState, '/multi_motor/states_deg', 10)
        self._pub_restored_joint_states = self.create_publisher(
            JointState, self._restored_joint_states_topic, 10)

        self.create_subscription(
            Float64MultiArray,
            '/multi_motor/pp_position_deg/commands',
            lambda msg: self._convert_position_command(
                msg, self._pub_pp_counts, 'pp_position_deg'),
            10)
        self.create_subscription(
            Float64MultiArray,
            '/multi_motor/csp_position_deg/commands',
            lambda msg: self._convert_position_command(
                msg, self._pub_csp_counts, 'csp_position_deg'),
            10)
        self.create_subscription(
            Float64MultiArray,
            '/multi_motor/csv_velocity_deg_s/commands',
            lambda msg: self._convert_velocity_command(
                msg, self._pub_csv_counts, 'csv_velocity_deg_s'),
            10)
        self.create_subscription(
            Float64MultiArray,
            '/multi_motor/pv_velocity_deg_s/commands',
            lambda msg: self._convert_velocity_command(
                msg, self._pub_pv_counts, 'pv_velocity_deg_s'),
            10)
        self.create_subscription(
            JointState,
            self._raw_joint_states_topic,
            self._handle_raw_joint_states,
            10)
        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self._handle_dynamic_joint_states,
            10)
        self.create_subscription(
            Float64MultiArray,
            '/multi_motor/reset_encoder_restore',
            self._handle_reset_encoder_restore,
            10)

        self.get_logger().info(
            'degree converter ready: encoder_resolution='
            f'{self._encoder_resolution:g} counts/output-rev, '
            f'joints={self._joint_names}')
        self.get_logger().info(
            'encoder restore ready: one_turn_cnt='
            f'{self._encoder_one_turn_cnt}, state_file='
            f'{self._encoder_state_file}, raw_topic='
            f'{self._raw_joint_states_topic}, restored_topic='
            f'{self._restored_joint_states_topic}, require_status_word='
            f'{self._encoder_require_status_word}, realign_threshold='
            f'{self._encoder_realign_threshold_counts}, status_timeout='
            f'{self._encoder_status_timeout_sec}, realign_holdoff='
            f'{self._encoder_realign_save_holdoff_sec}, powerup_wait='
            f'{self._encoder_powerup_stabilize_sec}, stable_duration='
            f'{self._encoder_raw_stable_duration_sec}, stable_delta='
            f'{self._encoder_raw_stable_delta_counts}')
        if self._position_limits_path is not None:
            self.get_logger().info(
                f'loaded position limits from {self._position_limits_path}')
        else:
            self.get_logger().warn(
                'no calibrate.json found; position commands are not limited')

    def _load_encoder_state_file(self) -> Dict[str, dict]:
        try:
            with self._encoder_state_file.open('r', encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().info(
                f'encoder state file not found: {self._encoder_state_file}')
            return {}
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(
                f'failed to load encoder state file '
                f'{self._encoder_state_file}: {exc}')
            return {}

        joints = data.get('joints', {})
        if not isinstance(joints, dict):
            self.get_logger().error(
                f'encoder state file {self._encoder_state_file} has no '
                'valid "joints" object')
            return {}
        return joints

    def _write_encoder_state_file(self) -> None:
        if not self._all_encoder_raw_counts_nonzero():
            return
        self._encoder_state_file.parent.mkdir(parents=True, exist_ok=True)
        now = time.time()
        data = {
            'version': 1,
            'timestamp': datetime.fromtimestamp(
                now, tz=timezone.utc).isoformat(),
            'timestamp_sec': now,
            'one_turn_cnt': self._encoder_one_turn_cnt,
            'joints': {
                joint_name: state.as_dict()
                for joint_name, state in self._encoder_states.items()
            },
        }
        tmp_path = self._encoder_state_file.with_name(
            f'.{self._encoder_state_file.name}.tmp')
        try:
            with tmp_path.open('w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, sort_keys=True)
                f.write('\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self._encoder_state_file)
            dir_fd = os.open(self._encoder_state_file.parent, os.O_DIRECTORY)
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
        except OSError as exc:
            self.get_logger().error(
                f'failed to save encoder state file '
                f'{self._encoder_state_file}: {exc}')
            try:
                tmp_path.unlink()
            except OSError:
                pass
            return

        for state in self._encoder_states.values():
            state.mark_saved()

    def _maybe_save_encoder_states(self, force: bool = False) -> None:
        now = time.monotonic()
        if self._encoder_save_suppressed:
            if self._all_encoder_feedback_ready():
                reason = self._encoder_save_suppression_reason or 'unknown'
                self._encoder_save_suppressed = False
                self._encoder_save_suppression_reason = ''
                self._reported_save_suppressed = False
                self.get_logger().info(
                    'encoder state auto-save resumed: all joints report '
                    f'fresh, stable feedback again (was suppressed: {reason})')
            else:
                if not self._reported_save_suppressed:
                    reason = self._encoder_save_suppression_reason or 'unknown'
                    self.get_logger().warn(
                        'encoder state auto-save is suppressed after encoder '
                        f'feedback became unreliable ({reason}); keeping the '
                        'existing JSON file unchanged until feedback is '
                        'fresh and stable again')
                    self._reported_save_suppressed = True
                return
        if now < self._encoder_save_holdoff_until:
            return
        if not self._all_encoder_feedback_ready():
            return
        if not self._all_encoder_raw_counts_nonzero():
            return
        if not all(state.valid for state in self._encoder_states.values()):
            return
        if not force:
            if (
                now - self._last_encoder_state_save_time <
                self._encoder_state_save_period_sec
            ):
                return
            if not any(
                state.needs_save(self._encoder_state_min_delta_counts)
                for state in self._encoder_states.values()
            ):
                return

        self._write_encoder_state_file()
        self._last_encoder_state_save_time = now

    def _reload_position_limits(self) -> bool:
        try:
            limits, path = load_joint_limits(
                self._position_limits_file, self._joint_geometry_file)
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().warn(
                f'failed to reload position limits from '
                f'{self._position_limits_file or "default path"}; keeping '
                f'previous limits: {exc}')
            return False
        if path != self._position_limits_path:
            self.get_logger().info(f'position limits reloaded from {path}')
        self._position_limits = limits
        self._position_limits_path = path
        return True

    def _encoder_one_turn_count_for_joint(self, joint_name: str) -> int:
        limit = self._position_limits.get(joint_name)
        if limit is not None and limit.encoder_one_turn_cnt > 0:
            return int(limit.encoder_one_turn_cnt)
        return self._encoder_one_turn_cnt

    def _handle_reset_encoder_restore(self, msg: Float64MultiArray) -> None:
        raw_vec = self._validate_vector(
            msg.data, 'reset_encoder_restore')
        if not raw_vec:
            return

        self._reload_position_limits()
        reset_joints = []
        for joint_name, raw_value in zip(self._joint_names, raw_vec):
            if not math.isfinite(raw_value):
                continue
            state = self._encoder_states.get(joint_name)
            if state is None:
                continue
            raw_cnt = int(round(raw_value))
            state.reset_restore_to_raw(raw_cnt)
            self._raw_stabilizing[joint_name] = False
            self._raw_stable_since[joint_name] = time.monotonic()
            self._raw_stability_last_cnt[joint_name] = raw_cnt
            self._encoder_realign_armed[joint_name] = False
            self._reported_stabilizing.discard(joint_name)
            self._reported_invalid_status.discard(joint_name)
            reset_joints.append(f'{joint_name}={raw_cnt}')

        if not reset_joints:
            return

        self._encoder_save_holdoff_until = 0.0
        self._encoder_save_suppressed = False
        self._encoder_save_suppression_reason = ''
        self._reported_save_suppressed = False
        self._reported_zero_raw_save_skip = False
        self._write_encoder_state_file()
        self._last_encoder_state_save_time = time.monotonic()
        self.get_logger().warn(
            'encoder restore reset after zero calibration: '
            f'{", ".join(reset_joints)}')

    def _validate_vector(self, data: Iterable[float], label: str) -> List[float]:
        vec = [float(x) for x in data]
        if len(vec) != self._num_joints:
            self.get_logger().warn(
                f'Ignoring {label}: expected {self._num_joints} values, '
                f'got {len(vec)}')
            return []
        return vec

    def _deg_to_counts(self, value_deg: float) -> float:
        counts = self._deg_to_counts_unclamped(value_deg)
        return float(self._clamp_counts(counts, 'Command'))

    def _deg_to_counts_unclamped(self, value_deg: float) -> int:
        return int(round(value_deg * self._counts_per_degree))

    def _deg_to_counts_for_joint(
            self, joint_name: str, value_deg: float) -> float:
        limit = self._position_limits.get(joint_name)
        resolution = (
            limit.encoder_resolution
            if limit is not None and not limit.is_linear else
            self._encoder_resolution)
        counts = int(round(float(value_deg) * resolution / 360.0))
        return float(self._clamp_counts(
            counts, f'{joint_name} rotary command'))

    def _clamp_counts(self, counts: int, label: str) -> int:
        if counts < INT32_MIN or counts > INT32_MAX:
            clamped = min(max(counts, INT32_MIN), INT32_MAX)
            self.get_logger().warn(
                f'{label} {counts} counts exceeds int32; '
                f'clamped to {clamped}')
            counts = clamped
        return counts

    def _counts_to_deg(
            self, value_counts: float, joint_name: Optional[str] = None
    ) -> float:
        limit = (
            self._position_limits.get(joint_name)
            if joint_name is not None else None)
        if limit is not None and not limit.is_linear:
            return limit.counts_to_raw_deg(
                value_counts, self._encoder_resolution)
        return float(value_counts) * self._degree_per_count

    def _position_unit(self, joint_name: str) -> str:
        limit = self._position_limits.get(joint_name)
        return limit.display_unit if limit is not None else 'deg'

    def _counts_to_position_value(
            self, joint_name: str, value_counts: float) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None:
            return self._counts_to_deg(value_counts)
        return limit.raw_counts_to_calibrated(
            value_counts,
            fallback_encoder_resolution=self._encoder_resolution)

    def _velocity_counts_to_value(
            self, joint_name: str, value_counts_s: float) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None or not limit.is_linear:
            return self._counts_to_deg(value_counts_s, joint_name)
        return (
            float(value_counts_s) *
            limit.linear_direction *
            limit.screw_lead_mm_per_rev /
            1000.0
        )

    def _raw_position_to_calibrated(
            self, joint_name: str, raw_deg: float) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None:
            return float(raw_deg)
        return limit.raw_to_calibrated(raw_deg)

    def _calibrated_position_to_raw(
            self, joint_name: str, calibrated_deg: float) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None:
            return float(calibrated_deg)
        return limit.calibrated_to_raw(calibrated_deg)

    def _limit_position_deg(
            self, joint_name: str, value_deg: float, label: str) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None:
            return float(value_deg)

        limited = limit.clamp(value_deg)
        if limited != float(value_deg):
            key = (label, joint_name)
            if key not in self._reported_limit_clamps:
                unit = limit.display_unit
                self.get_logger().warn(
                    f'{label}: {joint_name} command {float(value_deg):+.3f} '
                    f'{unit} outside [{limit.min_deg:+.3f}, '
                    f'{limit.max_deg:+.3f}] {unit}; clamped to '
                    f'{limited:+.3f} {unit}')
                self._reported_limit_clamps.add(key)
        return limited

    def _publish_array(self, pub, vec: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = vec
        pub.publish(msg)

    def _restored_position_to_drive_counts(
            self, joint_name: str, value_deg: float, label: str
    ) -> Optional[float]:
        state = self._encoder_states.get(joint_name)
        ready = (
            state is not None and state.valid and
            self._encoder_feedback_ready(joint_name))
        report_key = (label, joint_name)
        if not ready:
            if report_key not in self._reported_position_restore_not_ready:
                self.get_logger().warn(
                    f'Ignoring {label}: restored encoder state for '
                    f'{joint_name} is not ready, so restored-position '
                    'commands cannot be converted to drive raw counts yet')
                self._reported_position_restore_not_ready.add(report_key)
            return None

        self._reported_position_restore_not_ready.discard(report_key)
        limit = self._position_limits.get(joint_name)
        if limit is None:
            restored_target_cnt = self._deg_to_counts_unclamped(value_deg)
        else:
            restored_target_cnt = int(round(
                limit.calibrated_to_raw_counts(
                    value_deg, self._encoder_resolution)))
        drive_target_cnt = restored_target_cnt - state.restore_offset_cnt
        return float(self._clamp_counts(
            drive_target_cnt, f'{label}: {joint_name} target'))

    def _convert_position_command(
            self, msg: Float64MultiArray, pub, label: str) -> None:
        deg_vec = self._validate_vector(msg.data, label)
        if not deg_vec:
            return
        limited_vec = [
            self._limit_position_deg(joint_name, value, label)
            for joint_name, value in zip(self._joint_names, deg_vec)
        ]
        counts_vec: List[float] = []
        for joint_name, value in zip(self._joint_names, limited_vec):
            counts = self._restored_position_to_drive_counts(
                joint_name, value, label)
            if counts is None:
                return
            counts_vec.append(counts)
        self._publish_array(pub, counts_vec)

    def _convert_velocity_command(
            self, msg: Float64MultiArray, pub, label: str) -> None:
        deg_s_vec = self._validate_vector(msg.data, label)
        if not deg_s_vec:
            return
        self._publish_array(pub, [
            self._velocity_value_to_counts(joint_name, value, label)
            for joint_name, value in zip(self._joint_names, deg_s_vec)
        ])

    def _velocity_value_to_counts(
            self, joint_name: str, value: float, label: str) -> float:
        limit = self._position_limits.get(joint_name)
        if limit is None or not limit.is_linear:
            return self._deg_to_counts_for_joint(joint_name, value)
        raw_velocity = int(round(
            float(value) * 1000.0 /
            (limit.linear_direction * limit.screw_lead_mm_per_rev)))
        return float(self._clamp_counts(
            raw_velocity, f'{label}: {joint_name} velocity'))

    def _handle_dynamic_joint_states(self, msg: DynamicJointState) -> None:
        for joint_name, ifv in zip(msg.joint_names, msg.interface_values):
            if joint_name not in self._status_words:
                continue
            try:
                status_index = ifv.interface_names.index('status_word')
            except ValueError:
                continue
            if status_index >= len(ifv.values):
                continue

            status_value = ifv.values[status_index]
            now = time.monotonic()
            if math.isfinite(status_value):
                status_word = int(round(float(status_value)))
                was_valid = self._status_word_valid.get(joint_name, False)
                self._status_words[joint_name] = status_word
                self._status_word_valid[joint_name] = status_word != 0
                self._status_word_times[joint_name] = now
                if status_word != 0:
                    self._reported_invalid_status.discard(joint_name)
                    if not was_valid:
                        self._begin_raw_stabilization(joint_name, now)
                else:
                    self._mark_raw_unstable(joint_name)
            else:
                self._status_words[joint_name] = None
                self._status_word_valid[joint_name] = False
                self._status_word_times[joint_name] = now
                self._mark_raw_unstable(joint_name)

    def _begin_raw_stabilization(self, joint_name: str, now: float) -> None:
        self._raw_stabilizing[joint_name] = True
        self._raw_stabilize_not_before[joint_name] = (
            now + self._encoder_powerup_stabilize_sec)
        self._raw_stable_since[joint_name] = 0.0
        self._raw_stability_last_cnt[joint_name] = None
        self._reported_stabilizing.discard(joint_name)

    def _mark_raw_unstable(self, joint_name: str) -> None:
        self._raw_stabilizing[joint_name] = True
        self._raw_stable_since[joint_name] = 0.0
        self._raw_stability_last_cnt[joint_name] = None
        self._encoder_realign_armed[joint_name] = True
        self._suppress_encoder_state_saves(
            f'{joint_name} status_word became invalid or stale')
        self._encoder_save_holdoff_until = max(
            self._encoder_save_holdoff_until,
            time.monotonic() + self._encoder_realign_save_holdoff_sec)
        self._reported_stabilizing.discard(joint_name)

    def _encoder_status_fresh_valid(self, joint_name: str) -> bool:
        if not self._encoder_require_status_word:
            return True
        if not self._status_word_valid.get(joint_name, False):
            return False
        age = time.monotonic() - self._status_word_times.get(joint_name, 0.0)
        if age <= self._encoder_status_timeout_sec:
            return True
        self._status_word_valid[joint_name] = False
        self._mark_raw_unstable(joint_name)
        return False

    def _raw_position_stable_ready(
            self, joint_name: str, raw_cnt: int, now: float) -> bool:
        if not self._raw_stabilizing.get(joint_name, False):
            return True

        last_cnt = self._raw_stability_last_cnt.get(joint_name)
        if (
            last_cnt is None or
            abs(raw_cnt - last_cnt) > self._encoder_raw_stable_delta_counts
        ):
            self._raw_stability_last_cnt[joint_name] = raw_cnt
            self._raw_stable_since[joint_name] = now
            return False

        self._raw_stability_last_cnt[joint_name] = raw_cnt
        stable_since = self._raw_stable_since.get(joint_name, 0.0)
        if stable_since <= 0.0:
            self._raw_stable_since[joint_name] = now
            return False
        if now < self._raw_stabilize_not_before.get(joint_name, 0.0):
            return False
        if now - stable_since < self._encoder_raw_stable_duration_sec:
            return False

        self._raw_stabilizing[joint_name] = False
        self._reported_stabilizing.discard(joint_name)
        self.get_logger().info(
            f'{joint_name}: raw encoder position is stable at {raw_cnt} cnt')
        return True

    def _encoder_feedback_ready(self, joint_name: str) -> bool:
        return (
            self._encoder_status_fresh_valid(joint_name) and
            not self._raw_stabilizing.get(joint_name, False))

    def _dynamic_realign_allowed(self, joint_name: str) -> bool:
        if not self._encoder_require_status_word:
            return True
        return self._encoder_realign_armed.get(joint_name, False)

    def _all_encoder_feedback_ready(self) -> bool:
        return all(
            self._encoder_feedback_ready(joint_name)
            for joint_name in self._joint_names)

    def _all_encoder_raw_counts_nonzero(self) -> bool:
        zero_joints = [
            joint_name
            for joint_name, state in self._encoder_states.items()
            if state.raw_6064_cnt == 0
        ]
        if not zero_joints:
            self._reported_zero_raw_save_skip = False
            return True
        if not self._reported_zero_raw_save_skip:
            self.get_logger().warn(
                'encoder state auto-save skipped because raw_6064_cnt is '
                f'zero for {zero_joints}; keeping the existing JSON file '
                'unchanged')
            self._reported_zero_raw_save_skip = True
        return False

    def _suppress_encoder_state_saves(self, reason: str) -> None:
        self._encoder_save_suppressed = True
        if not self._encoder_save_suppression_reason:
            self._encoder_save_suppression_reason = reason

    def _handle_raw_joint_states(self, msg: JointState) -> None:
        self._update_encoder_restore(msg)
        self._publish_restored_joint_states(msg)
        self._publish_feedback_deg(msg)
        self._maybe_save_encoder_states()

    def _update_encoder_restore(self, msg: JointState) -> None:
        for i, joint_name in enumerate(msg.name):
            if joint_name not in self._encoder_states:
                continue
            now = time.monotonic()
            if not self._encoder_status_fresh_valid(joint_name):
                if joint_name not in self._reported_invalid_status:
                    self.get_logger().warn(
                        f'{joint_name}: status_word is zero or unavailable; '
                        'encoder restore state is held and will not be saved')
                    self._reported_invalid_status.add(joint_name)
                continue
            if i >= len(msg.position) or math.isnan(msg.position[i]):
                continue
            raw_cnt = int(round(float(msg.position[i])))
            state = self._encoder_states[joint_name]
            if not state.valid and raw_cnt == 0:
                if joint_name not in self._reported_zero_raw_init:
                    self.get_logger().warn(
                        f'{joint_name}: raw_6064_cnt is 0 before the first '
                        'encoder restore; holding restore until a nonzero '
                        'raw encoder value arrives')
                    self._reported_zero_raw_init.add(joint_name)
                continue
            self._reported_zero_raw_init.discard(joint_name)
            if (
                state.valid and
                not self._raw_stabilizing.get(joint_name, False) and
                abs(raw_cnt - state.raw_6064_cnt) >
                self._encoder_realign_threshold_counts
            ):
                # A jump this large between consecutive accepted samples
                # cannot be real motion; it is a drive-side rebase (power
                # dip drops 0x6064 to 0 a few ms before status_word goes
                # invalid). Hold the pre-jump continuous count as the
                # realign anchor instead of accepting the sample.
                self._suppress_encoder_state_saves(
                    f'{joint_name} raw encoder jump candidate')
                self._encoder_save_holdoff_until = max(
                    self._encoder_save_holdoff_until,
                    now + self._encoder_realign_save_holdoff_sec)
                self._encoder_realign_armed[joint_name] = True
                self._begin_raw_stabilization(joint_name, now)
                self.get_logger().warn(
                    f'{joint_name}: raw encoder jump candidate from '
                    f'{state.raw_6064_cnt} to {raw_cnt} cnt; holding the '
                    'previous restored position and waiting for the raw '
                    'position to stabilize before re-alignment')
            allow_realign = self._dynamic_realign_allowed(joint_name)

            if not self._raw_position_stable_ready(joint_name, raw_cnt, now):
                if joint_name not in self._reported_stabilizing:
                    self.get_logger().info(
                        f'{joint_name}: waiting for raw encoder position to '
                        'stabilize after power-up')
                    self._reported_stabilizing.add(joint_name)
                continue
            was_first_update = not state.valid
            realign_info = state.update(
                raw_cnt,
                self._encoder_realign_threshold_counts,
                allow_realign=allow_realign)
            self._encoder_realign_armed[joint_name] = False
            if was_first_update:
                if state.restored_from_saved:
                    self.get_logger().info(
                        f'{joint_name}: restored encoder state from file: '
                        f'raw={raw_cnt} cnt, restore_offset='
                        f'{state.restore_offset_cnt} cnt, continuous='
                        f'{state.continuous_cnt} cnt, restore_error_at_boot='
                        f'{state.restore_error_at_boot_cnt} cnt')
                else:
                    self.get_logger().info(
                        f'{joint_name}: no saved encoder state; starting '
                        f'from raw={raw_cnt} cnt with restore_offset=0')
            if realign_info is not None:
                self._encoder_save_holdoff_until = max(
                    self._encoder_save_holdoff_until,
                    time.monotonic() +
                    self._encoder_realign_save_holdoff_sec)
                self.get_logger().warn(
                    f'{joint_name}: raw encoder jump from '
                    f'{realign_info["raw_before_cnt"]} to '
                    f'{realign_info["raw_after_cnt"]} cnt '
                    f'(delta={realign_info["raw_delta_cnt"]}, '
                    f'nearest_turns={realign_info["raw_delta_turns"]}, '
                    f'residual={realign_info["raw_delta_residual_cnt"]}); '
                    f'realigned restore_offset_cnt='
                    f'{realign_info["restore_offset_cnt"]}, '
                    f'restored_delta_after_realign='
                    f'{realign_info["realign_delta_cnt"]} cnt')

    def _publish_restored_joint_states(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        effort = []
        for i, joint_name in enumerate(msg.name):
            state = self._encoder_states.get(joint_name)
            if state is not None:
                if state.valid and self._encoder_feedback_ready(joint_name):
                    effort.append(float(state.continuous_cnt))
                else:
                    effort.append(math.nan)
            elif i < len(msg.effort):
                effort.append(float(msg.effort[i]))
            else:
                effort.append(math.nan)
        out.effort = effort
        self._pub_restored_joint_states.publish(out)

    def _publish_feedback_deg(self, msg: JointState) -> None:
        pos_by_name = {
            name: msg.position[i]
            for i, name in enumerate(msg.name)
            if i < len(msg.position)
        }
        vel_by_name = {
            name: msg.velocity[i]
            for i, name in enumerate(msg.name)
            if i < len(msg.velocity)
        }

        out = DynamicJointState()
        out.header = msg.header
        out.joint_names = list(self._joint_names)

        for joint_name in self._joint_names:
            pos_counts = pos_by_name.get(joint_name, math.nan)
            vel_counts_s = vel_by_name.get(joint_name, math.nan)
            state = self._encoder_states.get(joint_name)
            unit = self._position_unit(joint_name)
            position_name = 'position_mm' if unit == 'mm' else 'position_deg'
            velocity_name = (
                'velocity_mm_s' if unit == 'mm' else 'velocity_deg_s')
            restored_position_name = (
                'restored_position_mm'
                if unit == 'mm' else 'restored_position_deg')
            restored_valid = (
                state is not None and state.valid and
                self._encoder_feedback_ready(joint_name))
            restored_counts = (
                state.continuous_cnt
                if restored_valid else math.nan)
            restore_offset_counts = (
                state.restore_offset_cnt
                if restored_valid else math.nan)
            turn = (
                state.turn
                if restored_valid else math.nan)
            single_counts = (
                state.single_cnt
                if restored_valid else math.nan)

            ifv = InterfaceValue()
            ifv.interface_names = [
                position_name,
                velocity_name,
                'raw_position_cnt',
                'restored_position_cnt',
                restored_position_name,
                'restore_offset_cnt',
                'turn',
                'single_cnt',
            ]
            pos_value = self._counts_to_position_value(
                joint_name, pos_counts)
            restored_value = self._counts_to_position_value(
                joint_name, restored_counts)
            ifv.values = [
                pos_value
                if not math.isnan(pos_counts) else math.nan,
                self._velocity_counts_to_value(joint_name, vel_counts_s)
                if not math.isnan(vel_counts_s) else math.nan,
                float(pos_counts)
                if not math.isnan(pos_counts) else math.nan,
                float(restored_counts)
                if not math.isnan(restored_counts) else math.nan,
                restored_value
                if not math.isnan(restored_counts) else math.nan,
                float(restore_offset_counts)
                if not math.isnan(restore_offset_counts) else math.nan,
                float(turn)
                if not math.isnan(turn) else math.nan,
                float(single_counts)
                if not math.isnan(single_counts) else math.nan,
            ]
            out.interface_values.append(ifv)

        self._pub_state_deg.publish(out)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
