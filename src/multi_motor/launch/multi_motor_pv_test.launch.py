#!/usr/bin/env python3
"""
multi_motor_pv_test.launch.py
-----------------------------
*** DIAGNOSTIC LAUNCH - DO NOT USE FOR PRODUCTION ***

Boots the same multi-motor stack as `multi_motor_control.launch.py`
but only loads the PV motion controller. It uses the normal
`ethercat_system.yaml` so the runtime configuration stays in one place.

It does NOT spawn `pp_controller` / `csp_controller` / `csv_controller`.
Only `pv_controller` is loaded (still INACTIVE - GUI activates it).

Purpose
-------
This is useful when you want to keep the controller-manager surface small
while checking PV behavior. For DC-off experiments, change
`assign_activate` in `ethercat_system.yaml` temporarily and change it back
before using CSP/CSV.

Usage
-----
  ros2 launch multi_motor_control multi_motor_pv_test.launch.py
  # then in another shell, while it runs:
  bash scripts/ec_diag.sh 60
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('multi_motor_control')

    declared = [
        DeclareLaunchArgument('num_joints',    default_value='4'),
        DeclareLaunchArgument('master_id',     default_value='0'),
        DeclareLaunchArgument('urdf_file',
            default_value='multi_motor.urdf.xacro'),
        DeclareLaunchArgument('controllers_file',
            default_value='controllers.yaml'),
        DeclareLaunchArgument('slave_config_file',
            default_value='ethercat_system.yaml',
            description='slave config filename under <pkg>/config/'),
    ]

    urdf_path = PathJoinSubstitution([
        pkg, 'urdf', LaunchConfiguration('urdf_file')])
    controllers_path = PathJoinSubstitution([
        pkg, 'config', LaunchConfiguration('controllers_file')])
    slave_config_path = PathJoinSubstitution([
        pkg, 'config', LaunchConfiguration('slave_config_file')])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_path,
        ' num_joints:=',   LaunchConfiguration('num_joints'),
        ' master_id:=',    LaunchConfiguration('master_id'),
        ' slave_config:=', slave_config_path,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content,
                                            value_type=str),
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_path],
        output='screen',
        emulate_tty=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    def spawn(name: str, *extra_args: str) -> Node:
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name,
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '30',
                       *extra_args],
            output='screen',
        )

    jsb_spawner         = spawn('joint_state_broadcaster')
    cia402_cmd_spawner  = spawn('cia402_cmd_controller')
    cia402_mode_spawner = spawn('cia402_mode_controller')
    fault_reset_spawner = spawn('fault_reset_controller')

    # Only PV - PP / CSP / CSV are intentionally NOT spawned in this
    # reduced diagnostic launch.
    pv_spawner = spawn('pv_controller', '--inactive')

    delay_cia402_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner,
                      on_exit=[cia402_cmd_spawner,
                               cia402_mode_spawner,
                               fault_reset_spawner]))

    delay_pv_after_cia402 = RegisterEventHandler(
        OnProcessExit(target_action=fault_reset_spawner,
                      on_exit=[pv_spawner]))

    banner = RegisterEventHandler(
        OnProcessExit(target_action=pv_spawner,
                      on_exit=[
            LogInfo(msg='=========================================='),
            LogInfo(msg='[PV-TEST] Only pv_controller is available.'),
            LogInfo(msg='[PV-TEST] PP / CSP / CSV are not loaded.'),
            LogInfo(msg='=========================================='),
        ]))

    return LaunchDescription(declared + [
        control_node,
        robot_state_publisher,
        jsb_spawner,
        delay_cia402_after_jsb,
        delay_pv_after_cia402,
        banner,
    ])
