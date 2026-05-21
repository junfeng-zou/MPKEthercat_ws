#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-level unit converter for the multi-motor CiA 402 stack.

The EtherCAT/ros2_control side intentionally stays in drive-native
counts/counts/s. This node exposes degree-based command topics for
higher-level software and republishes feedback in degree units.
"""

from __future__ import annotations

import math
from typing import Iterable, List

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


INT32_MIN = -2147483648
INT32_MAX = 2147483647
MOTOR_ENCODER_RESOLUTION = 131072.0  # 2^17 counts/motor rev
GEAR_RATIO = 6.6
DEFAULT_ENCODER_RESOLUTION = MOTOR_ENCODER_RESOLUTION * GEAR_RATIO


class UnitConverter(Node):
    """Bridge public degree topics to the lower-level count topics."""

    def __init__(self) -> None:
        super().__init__('multi_motor_unit_converter')

        self.declare_parameter('num_joints', 4)
        self.declare_parameter('encoder_resolution', DEFAULT_ENCODER_RESOLUTION)

        self._num_joints = int(self.get_parameter('num_joints').value)
        self._encoder_resolution = float(
            self.get_parameter('encoder_resolution').value)
        self.declare_parameter(
            'joint_names',
            [f'joint_{i}' for i in range(1, self._num_joints + 1)])
        configured_names = list(self.get_parameter('joint_names').value)
        self._joint_names = configured_names
        self._num_joints = len(self._joint_names)

        if self._encoder_resolution <= 0.0:
            raise ValueError('encoder_resolution must be positive')

        self._counts_per_degree = self._encoder_resolution / 360.0
        self._degree_per_count = 360.0 / self._encoder_resolution

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
            JointState, '/joint_states', self._publish_feedback_deg, 10)

        self.get_logger().info(
            'degree converter ready: encoder_resolution='
            f'{self._encoder_resolution:g} counts/output-rev, '
            f'joints={self._joint_names}')

    def _validate_vector(self, data: Iterable[float], label: str) -> List[float]:
        vec = [float(x) for x in data]
        if len(vec) != self._num_joints:
            self.get_logger().warn(
                f'Ignoring {label}: expected {self._num_joints} values, '
                f'got {len(vec)}')
            return []
        return vec

    def _deg_to_counts(self, value_deg: float) -> float:
        counts = int(round(value_deg * self._counts_per_degree))
        if counts < INT32_MIN or counts > INT32_MAX:
            clamped = min(max(counts, INT32_MIN), INT32_MAX)
            self.get_logger().warn(
                f'Command {counts} counts exceeds int32; clamped to {clamped}')
            counts = clamped
        return float(counts)

    def _counts_to_deg(self, value_counts: float) -> float:
        return float(value_counts) * self._degree_per_count

    def _publish_array(self, pub, vec: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = vec
        pub.publish(msg)

    def _convert_position_command(
            self, msg: Float64MultiArray, pub, label: str) -> None:
        deg_vec = self._validate_vector(msg.data, label)
        if not deg_vec:
            return
        self._publish_array(pub, [self._deg_to_counts(x) for x in deg_vec])

    def _convert_velocity_command(
            self, msg: Float64MultiArray, pub, label: str) -> None:
        deg_s_vec = self._validate_vector(msg.data, label)
        if not deg_s_vec:
            return
        self._publish_array(pub, [self._deg_to_counts(x) for x in deg_s_vec])

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

            ifv = InterfaceValue()
            ifv.interface_names = ['position_deg', 'velocity_deg_s']
            ifv.values = [
                self._counts_to_deg(pos_counts)
                if not math.isnan(pos_counts) else math.nan,
                self._counts_to_deg(vel_counts_s)
                if not math.isnan(vel_counts_s) else math.nan,
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
