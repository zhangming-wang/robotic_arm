import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def _load_yaml(package_name, relative_file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, relative_file_path)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def build_moveit_launch_description(
    bringup_launch_file,
    xacro_args,
    use_sim_time,
):
    arm_description_share = get_package_share_directory("arm_description")
    moveit_config_share = get_package_share_directory("arm_moveit")

    xacro_file = os.path.join(arm_description_share, "urdf", "arm_description.xacro")
    srdf_file = os.path.join(moveit_config_share, "config", "arm.srdf")

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file] + xacro_args)
    }

    with open(srdf_file, "r", encoding="utf-8") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            "arm_moveit", "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": _load_yaml(
            "arm_moveit", "config/joint_limits.yaml"
        )["joint_limits"]
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = _load_yaml("arm_moveit", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    trajectory_execution = _load_yaml("arm_moveit", "config/trajectory_execution.yaml")
    moveit_controllers = _load_yaml("arm_moveit", "config/moveit_controllers.yaml")
    planning_scene_monitor_parameters = _load_yaml(
        "arm_moveit", "config/planning_scene_monitor_parameters.yaml"
    )

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_description_share, "launch", bringup_launch_file)
        ),
        launch_arguments={
            "start_rviz": "false",
        }.items(),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=[
            "-d",
            os.path.join(moveit_config_share, "config", "display_settings.rviz"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            arm_bringup,
            move_group_node,
            rviz_node,
        ]
    )
