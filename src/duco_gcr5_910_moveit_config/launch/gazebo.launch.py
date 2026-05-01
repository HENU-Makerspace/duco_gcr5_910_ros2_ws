import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "duco_gcr5_910_moveit_config"
    package_share = get_package_share_directory(package_name)
    duco_support_share_parent = os.path.dirname(get_package_share_directory("duco_support"))
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([package_share, "config", "gcr5_910_gazebo.urdf.xacro"]),
            " initial_positions_file:=",
            LaunchConfiguration("initial_positions_file"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder("gcr5_910", package_name=package_name)
        .robot_description(
            file_path=os.path.join(package_share, "config", "gcr5_910_gazebo.urdf.xacro"),
            mappings={"initial_positions_file": LaunchConfiguration("initial_positions_file")},
        )
        .to_moveit_configs()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("gz_args"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "gcr5_910",
            "-allow_renaming",
            "false",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_1_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner],
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_moveit")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(package_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(package_share, "worlds", "gcr5_910_empty.sdf"),
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value=["-r -v 3 ", LaunchConfiguration("world")],
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=os.path.join(package_share, "config", "initial_positions_gazebo.yaml"),
            ),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[duco_support_share_parent, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            gz_sim,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            start_controllers,
            move_group,
            rviz,
        ]
    )
