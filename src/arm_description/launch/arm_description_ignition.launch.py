import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world = LaunchConfiguration("world", default="empty.sdf")

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

    gz_sim = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        arguments=["-r", world],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        name="robot_state_publisher",
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
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
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rqt_joint_trajectory_controller_node = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        name="rqt_joint_trajectory_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher_node,
            spawn_robot,
            load_joint_state_broadcaster,
            load_arm_controller,
            rqt_joint_trajectory_controller_node,
            rviz2_node,
        ]
    )
