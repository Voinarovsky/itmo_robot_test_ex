from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_type = LaunchConfiguration("robot_type")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gui = LaunchConfiguration("gui")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf/xacro/inc", description_file]
            ),
            " ",
            "aubo_type:=",
            robot_type,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    delay_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={"gui": gui, "verbose": "false"}.items(),
    )

    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_aubo",
        arguments=["-entity", "aubo", "-topic", "robot_description", "-z", "0.0"],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        gazebo,
        gazebo_spawn_robot,
        joint_state_broadcaster_spawner,
        delay_jtc_after_jsb,
        delay_rviz_after_jsb,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of used aubo robot.",
            choices=["aubo_i5", "aubo_C3", "aubo_i7", "aubo_iS7"],
            default_value="aubo_i7",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="aubo_description",
            description="Description package with robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo_gazebo.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            "gui", default_value="false", description="Launch Gazebo GUI?"
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])