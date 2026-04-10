import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.actions import LogInfo


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
    controller_config = os.path.join(pkg_share, "config", "arm_controllers.yaml")

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

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

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="screen",
    # )

    # joint_state_publisher_node=Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     output="screen",
    # )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
        ],
        parameters=[{"use_sim_time": False}],
        output="both",
    )

    # 激活 joint_state_broadcaster
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": False}],
    )

    # 激活 arm_controller
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{"use_sim_time": False}],
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_jsb],
        )
    )

    # 当 joint_state_broadcaster 激活成功退出后，再启动 arm_controller
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

    rqt_joint_trajectory_controller_node = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        name="rqt_joint_trajectory_controller",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    nodes_lis = [
        start_rviz_arg,
        robot_state_publisher_node,
        node_controller_manager,  # 必须显式加入这个节点
        load_joint_state_broadcaster,
        load_arm_controller,
        # rqt_joint_trajectory_controller_node,
        rviz2_node,
    ]

    return LaunchDescription(nodes_lis)
