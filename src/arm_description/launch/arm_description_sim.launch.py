import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    gazebo_world_path = os.path.join(pkg_share, "world", "empty.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "arm_description.xacro")
    rviz_config_path = os.path.join(pkg_share, "config", "display_settings.rviz")
    controller_config = os.path.join(pkg_share, "config", "arm_controllers.yaml")

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " use_sim:=true",
            ]
        )
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"{gazebo_world_path} -r"}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
        name="robot_state_publisher",
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="gazebo_create",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-name",
            "servo_arm",
            "-topic",
            "robot_description",
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
        parameters=[{"use_sim_time": True}],
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
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_jsb],
        )
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
        condition=IfCondition(start_rviz),
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # rqt_joint_trajectory_controller_node = Node(
    #     package="rqt_joint_trajectory_controller",
    #     executable="rqt_joint_trajectory_controller",
    #     name="rqt_joint_trajectory_controller",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    # )

    return LaunchDescription(
        [
            start_rviz_arg,
            gz_sim,
            robot_state_publisher_node,
            spawn_robot,
            load_joint_state_broadcaster,
            load_arm_controller,
            # rqt_joint_trajectory_controller_node,
            rviz2_node,
        ]
    )
