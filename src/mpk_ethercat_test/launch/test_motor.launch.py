import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mpk_ethercat_test'

    urdf_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'single_motor.urdf'
    )
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {"robot_description": robot_desc}

    controller_yaml = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'controller.yaml'
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_yaml],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    motor_data_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["MotorDataBroadcaster", "--controller-manager", "/controller_manager"],
    )

    control_word_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["control_word_controller", "--controller-manager", "/controller_manager"],
    )

    op_mode_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["op_mode_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        motor_data_broadcaster_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=motor_data_broadcaster_spawner,
                on_exit=[control_word_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_word_controller_spawner,
                on_exit=[op_mode_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=op_mode_controller_spawner,
                on_exit=[position_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=position_controller_spawner,
                on_exit=[velocity_controller_spawner],
            )
        ),
    ])