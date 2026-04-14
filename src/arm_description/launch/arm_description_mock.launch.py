import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Whether to start RViz in arm_description bringup.",
    )

    start_rviz = LaunchConfiguration("start_rviz", default="true")

    xacro_file = os.path.join(pkg_share, "urdf", "arm_description.xacro")
    rviz_config_path = os.path.join(pkg_share, "config", "display_settings.rviz")
    controller_config = os.path.join(pkg_share, "config", "arm_controllers_sim.yaml")

    robot_description = {
        "robot_description": Command(
            ["xacro ", xacro_file, " use_sim:=true", " use_mock_hardware:=true"]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
        name="robot_state_publisher",
        output="screen",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controller_config,
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controller_config,
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    load_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[spawn_jsb],
    )

    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm_controller],
        )
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription(
        [
            start_rviz_arg,
            robot_state_publisher_node,
            controller_manager_node,
            load_joint_state_broadcaster,
            load_arm_controller,
            rviz2_node,
        ]
    )
