#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Load calibrated joint position limits for degree-space commands."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Dict, Optional, Tuple


MOTOR_ENCODER_RESOLUTION = 131072.0
DEFAULT_ROTARY_GEAR_RATIO = 6.6
DEFAULT_ROTARY_ENCODER_RESOLUTION = (
    MOTOR_ENCODER_RESOLUTION * DEFAULT_ROTARY_GEAR_RATIO)
DEFAULT_ENCODER_ONE_TURN_CNT = int(MOTOR_ENCODER_RESOLUTION)
DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV = 131072.0
DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV = 10.0
DEFAULT_LINEAR_DIRECTION = 1.0
STATIC_GEOMETRY_KEYS = {
    'axis_type',
    'unit',
    'encoder_resolution',
    'encoder_one_turn_cnt',
    'encoder_counts_per_rev',
    'screw_lead_mm_per_rev',
    'linear_direction',
}


@dataclass(frozen=True)
class JointLimit:
    min_deg: float
    max_deg: float
    range_deg: float
    default_deg: float
    home_offset_deg: float = 0.0
    unit: str = 'deg'
    axis_type: str = 'rotary'
    encoder_resolution: float = DEFAULT_ROTARY_ENCODER_RESOLUTION
    encoder_one_turn_cnt: int = DEFAULT_ENCODER_ONE_TURN_CNT
    encoder_counts_per_rev: float = DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV
    screw_lead_mm_per_rev: float = DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV
    zero_position_cnt: Optional[float] = None
    linear_direction: float = DEFAULT_LINEAR_DIRECTION

    @property
    def is_linear(self) -> bool:
        return self.axis_type == 'linear' or self.unit == 'mm'

    @property
    def display_unit(self) -> str:
        return 'mm' if self.is_linear else 'deg'

    def clamp(self, value_deg: float) -> float:
        return min(max(float(value_deg), self.min_deg), self.max_deg)

    def raw_to_calibrated(self, raw_deg: float) -> float:
        return float(raw_deg) + self.home_offset_deg

    def calibrated_to_raw(self, calibrated_deg: float) -> float:
        return float(calibrated_deg) - self.home_offset_deg

    def counts_to_raw_deg(
            self,
            raw_counts: float,
            fallback_encoder_resolution: Optional[float] = None) -> float:
        resolution = (
            float(self.encoder_resolution)
            if self.encoder_resolution > 0.0 else
            float(fallback_encoder_resolution))
        return float(raw_counts) * 360.0 / resolution

    def raw_counts_to_calibrated(
            self,
            raw_counts: float,
            raw_deg: Optional[float] = None,
            fallback_encoder_resolution: Optional[float] = None) -> float:
        if not self.is_linear:
            if raw_deg is None:
                raw_deg = self.counts_to_raw_deg(
                    raw_counts, fallback_encoder_resolution)
            return self.raw_to_calibrated(raw_deg)
        zero_count = (
            float(self.zero_position_cnt)
            if self.zero_position_cnt is not None else 0.0)
        return (
            (float(raw_counts) - zero_count) *
            self.linear_direction *
            self.screw_lead_mm_per_rev /
            self.encoder_counts_per_rev
        )

    def calibrated_to_raw_counts(
            self,
            calibrated_value: float,
            encoder_resolution: Optional[float] = None) -> float:
        if not self.is_linear:
            resolution = (
                float(self.encoder_resolution)
                if self.encoder_resolution > 0.0 else
                float(encoder_resolution))
            return (
                self.calibrated_to_raw(calibrated_value) *
                resolution / 360.0
            )
        zero_count = (
            float(self.zero_position_cnt)
            if self.zero_position_cnt is not None else 0.0)
        return (
            zero_count +
            float(calibrated_value) *
            self.encoder_counts_per_rev /
            self.screw_lead_mm_per_rev *
            self.linear_direction
        )


def _candidate_paths(explicit_path: str = ''):
    if explicit_path:
        yield Path(explicit_path).expanduser()

    env_path = os.environ.get('MULTI_MOTOR_CALIBRATE_FILE', '').strip()
    if env_path:
        yield Path(env_path).expanduser()

    # Source-tree / symlink-install layout:
    #   src/multi_motor/multi_motor_control/joint_limits.py
    #   src/multi_motor/calibrate.json
    yield Path(__file__).resolve().parents[1] / 'calibrate.json'

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return

    try:
        yield Path(get_package_share_directory('multi_motor_control')) / (
            'calibrate.json')
    except Exception:
        return


def _geometry_candidate_paths(explicit_path: str = ''):
    if explicit_path:
        yield Path(explicit_path).expanduser()

    env_path = os.environ.get('MULTI_MOTOR_JOINT_GEOMETRY_FILE', '').strip()
    if env_path:
        yield Path(env_path).expanduser()

    yield Path(__file__).resolve().parents[1] / 'config' / (
        'joint_geometry.yaml')

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return

    try:
        yield Path(get_package_share_directory('multi_motor_control')) / (
            'config' / 'joint_geometry.yaml')
    except Exception:
        return


def _load_mapping_file(path: Path):
    suffix = path.suffix.lower()
    with path.open('r', encoding='utf-8') as f:
        if suffix == '.json':
            data = json.load(f)
        else:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError(
                    f'{path} is YAML but PyYAML is not installed') from exc
            data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a mapping/object')
    return data


def load_joint_geometry(
        explicit_path: str = '') -> Tuple[Dict[str, dict], Optional[Path]]:
    """Return static mechanical parameters keyed by joint name."""
    seen = set()
    for path in _geometry_candidate_paths(explicit_path):
        path = path.resolve()
        if path in seen:
            continue
        seen.add(path)
        if not path.is_file():
            continue

        raw = _load_mapping_file(path)
        joints = raw.get('joints', raw)
        if not isinstance(joints, dict):
            raise ValueError(f'{path}: "joints" must be a mapping/object')
        geometry = {}
        for joint_name, entry in joints.items():
            if not isinstance(entry, dict):
                raise ValueError(f'{path}: {joint_name} must be a mapping')
            geometry[str(joint_name)] = dict(entry)
        return geometry, path

    return {}, None


def _write_json_atomic(path: Path, data) -> None:
    """Write JSON without exposing a partially written calibrate file."""
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f'.{path.name}.tmp.{os.getpid()}')
    try:
        with tmp_path.open('w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
        try:
            dir_fd = os.open(path.parent, os.O_DIRECTORY)
        except OSError:
            return
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except Exception:
        try:
            tmp_path.unlink()
        except OSError:
            pass
        raise


def load_joint_limits(
        explicit_path: str = '',
        geometry_path: str = '') -> Tuple[Dict[str, JointLimit], Optional[Path]]:
    """Return calibrated limits keyed by joint name plus the file path used."""
    geometry, _ = load_joint_geometry(geometry_path)
    seen = set()
    for path in _candidate_paths(explicit_path):
        path = path.resolve()
        if path in seen:
            continue
        seen.add(path)
        if not path.is_file():
            continue

        with path.open('r', encoding='utf-8') as f:
            raw = json.load(f)
        if not isinstance(raw, dict):
            raise ValueError(f'{path} must contain a mapping/object')

        limits: Dict[str, JointLimit] = {}
        joint_names = list(geometry.keys())
        joint_names.extend(
            str(joint_name)
            for joint_name in raw.keys()
            if str(joint_name) not in geometry)

        for joint_name in joint_names:
            entry = raw.get(joint_name, {})
            geom_entry = geometry.get(joint_name, {})
            if not isinstance(entry, dict):
                raise ValueError(f'{path}: {joint_name} must be a mapping')

            def static_value(key: str, default=None):
                return geom_entry.get(key, entry.get(key, default))

            axis_type = str(static_value('axis_type', '')).strip().lower()
            unit = str(static_value('unit', '')).strip().lower()
            if not axis_type:
                axis_type = (
                    'linear'
                    if unit == 'mm' or 'min_mm' in entry else
                    'rotary')
            if not unit:
                unit = 'mm' if axis_type == 'linear' else 'deg'

            if unit == 'mm':
                min_value = float(entry.get('min_mm', entry.get('min_deg')))
                max_value = float(entry.get('max_mm', entry.get('max_deg')))
                range_value = float(entry.get(
                    'range_mm', entry.get(
                        'range_deg', max_value - min_value)))
                default_value = float(entry.get(
                    'default_mm', entry.get('default_deg', 0.0)))
            else:
                min_value = float(entry['min_deg'])
                max_value = float(entry['max_deg'])
                range_value = float(
                    entry.get('range_deg', max_value - min_value))
                default_value = float(entry['default_deg'])

            encoder_resolution = float(static_value(
                'encoder_resolution', DEFAULT_ROTARY_ENCODER_RESOLUTION))
            encoder_one_turn_cnt = int(round(float(static_value(
                'encoder_one_turn_cnt', DEFAULT_ENCODER_ONE_TURN_CNT))))
            zero_position_cnt = entry.get(
                'zero_position_cnt', entry.get('home_offset_cnt'))
            if zero_position_cnt is not None:
                zero_position_cnt = float(zero_position_cnt)

            limit = JointLimit(
                min_deg=min_value,
                max_deg=max_value,
                range_deg=range_value,
                default_deg=default_value,
                home_offset_deg=float(entry.get('home_offset_deg', 0.0)),
                unit=unit,
                axis_type=axis_type,
                encoder_resolution=encoder_resolution,
                encoder_one_turn_cnt=encoder_one_turn_cnt,
                encoder_counts_per_rev=float(static_value(
                    'encoder_counts_per_rev',
                    DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV)),
                screw_lead_mm_per_rev=float(static_value(
                    'screw_lead_mm_per_rev',
                    DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV)),
                zero_position_cnt=zero_position_cnt,
                linear_direction=float(static_value(
                    'linear_direction', DEFAULT_LINEAR_DIRECTION)),
            )
            if limit.min_deg > limit.max_deg:
                raise ValueError(
                    f'{joint_name}: min exceeds max in {path}')
            if limit.range_deg <= 0.0:
                raise ValueError(
                    f'{joint_name}: range must be positive in {path}')
            if not limit.min_deg <= limit.default_deg <= limit.max_deg:
                raise ValueError(
                    f'{joint_name}: default is outside limits in {path}')
            if limit.encoder_resolution <= 0.0:
                raise ValueError(
                    f'{joint_name}: encoder_resolution must be positive')
            if limit.encoder_one_turn_cnt <= 0:
                raise ValueError(
                    f'{joint_name}: encoder_one_turn_cnt must be positive')
            if limit.is_linear:
                if limit.encoder_counts_per_rev <= 0.0:
                    raise ValueError(
                        f'{joint_name}: encoder_counts_per_rev must be '
                        f'positive in {path}')
                if limit.screw_lead_mm_per_rev <= 0.0:
                    raise ValueError(
                        f'{joint_name}: screw_lead_mm_per_rev must be '
                        f'positive in {path}')
                if limit.linear_direction == 0.0:
                    raise ValueError(
                        f'{joint_name}: linear_direction cannot be zero '
                        f'in {path}')
            limits[str(joint_name)] = limit
        return limits, path

    return {}, None


def save_min_zero_calibration(
        path: Path,
        joint_names,
        raw_min_by_joint: Dict[str, float]) -> Dict[str, JointLimit]:
    """Set each homed joint's minimum to calibrated 0 and persist offsets."""
    path = Path(path).resolve()
    with path.open('r', encoding='utf-8') as f:
        raw = json.load(f)

    for joint_name in joint_names:
        if joint_name not in raw or joint_name not in raw_min_by_joint:
            continue

        entry = raw[joint_name]
        range_deg = float(
            entry.get('range_deg',
                      float(entry['max_deg']) - float(entry['min_deg'])))
        if range_deg <= 0.0:
            raise ValueError(f'{joint_name}: range_deg must be positive')
        raw_min_deg = float(raw_min_by_joint[joint_name])
        entry['min_deg'] = 0.0
        entry['max_deg'] = range_deg
        entry['range_deg'] = range_deg
        entry['default_deg'] = min(
            max(float(entry.get('default_deg', 0.0)), 0.0), range_deg)
        entry.pop('dead_zone_compensation_deg', None)
        entry.pop('default_calibrated', None)
        entry['home_offset_deg'] = -raw_min_deg

    _write_json_atomic(path, raw)

    limits, _ = load_joint_limits(str(path))
    return limits


def save_range_calibration(
        path: Path,
        joint_names,
        raw_min_by_joint: Dict[str, float],
        raw_max_by_joint: Dict[str, float]) -> Dict[str, JointLimit]:
    """Persist measured travel range without changing the calibrated zero."""
    path = Path(path).resolve()
    with path.open('r', encoding='utf-8') as f:
        raw = json.load(f)

    for joint_name in joint_names:
        if (
            joint_name not in raw or
            joint_name not in raw_min_by_joint or
            joint_name not in raw_max_by_joint
        ):
            continue

        raw_min_deg = float(raw_min_by_joint[joint_name])
        raw_max_deg = float(raw_max_by_joint[joint_name])
        range_deg = raw_max_deg - raw_min_deg
        if range_deg <= 0.0:
            raise ValueError(
                f'{joint_name}: measured max {raw_max_deg:+.3f} deg '
                f'must exceed min {raw_min_deg:+.3f} deg')

        entry = raw[joint_name]
        default_deg = float(entry.get('default_deg', 0.0))
        entry['min_deg'] = 0.0
        entry['max_deg'] = range_deg
        entry['range_deg'] = range_deg
        entry['default_deg'] = min(max(default_deg, 0.0), range_deg)
        entry.pop('dead_zone_compensation_deg', None)
        entry.pop('default_calibrated', None)

    _write_json_atomic(path, raw)

    limits, _ = load_joint_limits(str(path))
    return limits


def save_linear_manual_calibration(
        path: Path,
        joint_name: str,
        zero_position_cnt: float,
        range_mm: float,
        encoder_counts_per_rev: float = DEFAULT_LINEAR_ENCODER_COUNTS_PER_REV,
        screw_lead_mm_per_rev: float = DEFAULT_LINEAR_SCREW_LEAD_MM_PER_REV,
        linear_direction: float = DEFAULT_LINEAR_DIRECTION) -> Dict[str, JointLimit]:
    """Persist a manually swept ball-screw axis as 0..range_mm."""
    path = Path(path).resolve()
    with path.open('r', encoding='utf-8') as f:
        raw = json.load(f)

    if range_mm <= 0.0:
        raise ValueError(
            f'{joint_name}: measured linear range must be positive')
    if encoder_counts_per_rev <= 0.0:
        raise ValueError('encoder_counts_per_rev must be positive')
    if screw_lead_mm_per_rev <= 0.0:
        raise ValueError('screw_lead_mm_per_rev must be positive')
    if linear_direction == 0.0:
        raise ValueError('linear_direction cannot be zero')

    entry = raw.setdefault(joint_name, {})
    default_mm = float(entry.get(
        'default_mm', entry.get('default_deg', 0.0)))
    default_mm = min(max(default_mm, 0.0), float(range_mm))

    entry['min_mm'] = 0.0
    entry['max_mm'] = float(range_mm)
    entry['range_mm'] = float(range_mm)
    entry['default_mm'] = default_mm
    entry['zero_position_cnt'] = float(zero_position_cnt)
    for key in STATIC_GEOMETRY_KEYS:
        entry.pop(key, None)

    # Keep legacy field names present so older tools fail less abruptly.
    entry['min_deg'] = 0.0
    entry['max_deg'] = float(range_mm)
    entry['range_deg'] = float(range_mm)
    entry['default_deg'] = default_mm
    entry['home_offset_deg'] = 0.0
    entry.pop('dead_zone_compensation_deg', None)
    entry.pop('default_calibrated', None)

    _write_json_atomic(path, raw)

    limits, _ = load_joint_limits(str(path))
    return limits
