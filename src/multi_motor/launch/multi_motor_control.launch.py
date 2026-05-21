#!/usr/bin/env python3
"""
multi_motor_control.launch.py
-----------------------------
ROS 2 Humble launch for the Denali XCR multi-motor system.

Order of operations
  1. xacro -> robot_description   (with num_joints & slave_config as args)
  2. ros2_control_node            (loads hardware plugin + controllers.yaml)
  3. spawn joint_state_broadcaster        (active)
  4. spawn cia402_cmd_controller          (active, keeps CW=0 by default)
  5. spawn cia402_mode_controller         (active)
  6. spawn fault_reset_controller         (active)
  7. spawn pp/csp/csv/pv controllers      (INACTIVE - GUI will activate)
  8. unit_converter                       (degree API <-> drive counts)

Because the YAML sets Control Word default=0 AND cia402_cmd_controller
is active with no command published, the drives stay in
"Switch On Disabled" after bring-up, as required.
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

    # ---- arguments ----
    declared = [
        DeclareLaunchArgument('num_joints',    default_value='4'),
        DeclareLaunchArgument('master_id',     default_value='0'),
        DeclareLaunchArgument('urdf_file',
            default_value='multi_motor.urdf.xacro',
            description='xacro filename under <pkg>/urdf/'),
        DeclareLaunchArgument('controllers_file',
            default_value='controllers.yaml',
            description='controllers filename under <pkg>/config/'),
        DeclareLaunchArgument('slave_config_file',
            default_value='ethercat_system.yaml',
            description='slave config filename under <pkg>/config/'),
        DeclareLaunchArgument('encoder_resolution',
            default_value='865075.2',
            description='Counts per output revolution for degree conversion.'),
    ]

    # ---- absolute paths ----
    urdf_path = PathJoinSubstitution([
        pkg, 'urdf', LaunchConfiguration('urdf_file')])
    controllers_path = PathJoinSubstitution([
        pkg, 'config', LaunchConfiguration('controllers_file')])
    slave_config_path = PathJoinSubstitution([
        pkg, 'config', LaunchConfiguration('slave_config_file')])

    # ---- robot_description via xacro ----
    # NOTE: newer launch_ros tries to parse parameter values as YAML by
    # default, which breaks on URDF/XML. Wrap with ParameterValue and
    # force value_type=str so the xml is kept verbatim.
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

    # ---- nodes ----
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

    unit_converter = Node(
        package='multi_motor_control',
        executable='unit_converter',
        parameters=[{
            'num_joints': ParameterValue(
                LaunchConfiguration('num_joints'), value_type=int),
            'encoder_resolution': ParameterValue(
                LaunchConfiguration('encoder_resolution'), value_type=float),
        }],
        output='screen',
    )

    # ---- spawner helper ----
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

    # Always-on controllers -------------------------------
    jsb_spawner = spawn('joint_state_broadcaster')

    # Order matters: load the Control-Word writer FIRST so that
    # the default Control-Word default=0 from the YAML latches
    # before anything else touches the drive.
    cia402_cmd_spawner  = spawn('cia402_cmd_controller')
    cia402_mode_spawner = spawn('cia402_mode_controller')
    fault_reset_spawner = spawn('fault_reset_controller')

    # Motion controllers -- INACTIVE ---------------------
    pp_spawner  = spawn('pp_controller',  '--inactive')
    csp_spawner = spawn('csp_controller', '--inactive')
    csv_spawner = spawn('csv_controller', '--inactive')
    pv_spawner  = spawn('pv_controller',  '--inactive')

    # ---- serialise spawners so controller_manager has time to
    #       load all of them cleanly ----
    delay_cia402_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner,
                      on_exit=[cia402_cmd_spawner,
                               cia402_mode_spawner,
                               fault_reset_spawner]))

    delay_motion_after_cia402 = RegisterEventHandler(
        OnProcessExit(target_action=fault_reset_spawner,
                      on_exit=[pp_spawner, csp_spawner, csv_spawner, pv_spawner]))

    banner = RegisterEventHandler(
        OnProcessExit(target_action=pv_spawner,
                      on_exit=[
            LogInfo(msg='=========================================='),
            LogInfo(msg='All controllers loaded.'),
            LogInfo(msg='ROS degree API is available under /multi_motor/* '
                        '(encoder_resolution=865075.2 by default).'),
            LogInfo(msg='Drives are held in "Switch On Disabled" '
                        '(Control Word default=0).'),
            LogInfo(msg='Use rqt -> Robot -> Multi-Motor Control '
                        'to enable and drive them.'),
            LogInfo(msg='=========================================='),
        ]))

    return LaunchDescription(declared + [
        control_node,
        robot_state_publisher,
        unit_converter,
        jsb_spawner,
        delay_cia402_after_jsb,
        delay_motion_after_cia402,
        banner,
    ])
