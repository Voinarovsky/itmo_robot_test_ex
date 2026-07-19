from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):

    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_description"),
                 "urdf/xacro/inc/", "aubo_gazebo.xacro"]
            ),
            " ",
            "aubo_type:=aubo_i7 ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_moveit_config"), "config", "aubo_robot.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic":
            robot_description_semantic_content.perform(context)
    }

    kinematics_yaml = load_yaml("aubo_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = {
        "robot_description_planning":
            load_yaml("aubo_moveit_config", "config/joint_limits.yaml")
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("aubo_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "aubo_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            use_sim_time,
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("aubo_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            use_sim_time,
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Запускать ли RViz (в Docker без X11 держать false)",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
