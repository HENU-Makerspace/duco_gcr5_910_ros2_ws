import os
import yaml

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
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    package_name = "duco_gcr5_910_moveit_config"
    package_share = get_package_share_directory(package_name)
    duco_support_share_parent = os.path.dirname(get_package_share_directory("duco_support"))
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    gazebo_controllers_file = os.path.join(
        package_share, "config", "ros2_controllers_dual.yaml")
    moveit_controllers_file = os.path.join(
        package_share, "config", "moveit_controllers_dual.yaml")

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([package_share, "config", "gcr5_910_gazebo.urdf.xacro"]),
            " initial_positions_file:=",
            LaunchConfiguration("initial_positions_file"),
            " controllers_file:=",
            gazebo_controllers_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder("gcr5_910", package_name=package_name)
        .robot_description(
            file_path=os.path.join(package_share, "config", "gcr5_910_gazebo.urdf.xacro"),
            mappings={
                "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                "controllers_file": gazebo_controllers_file,
            },
        )
        .to_moveit_configs()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
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

    sim_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_arm_1_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_sim_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, sim_arm_controller_spawner],
        )
    )

    dual_trajectory_action = Node(
        package="duco_ros_driver",
        executable="DualTrajectoryAction",
        output="screen",
        parameters=[
            {"front_action_name": "arm_1_controller/follow_joint_trajectory"},
            {"sim_action_name": "sim_arm_1_controller/follow_joint_trajectory"},
            {"real_command_topic": "joint_path_command"},
            {"publish_real": True},
            {"sim_time_scale": LaunchConfiguration("sim_time_scale")},
        ],
    )

    duco_driver = Node(
        package="duco_ros_driver",
        executable="DucoDriver",
        output="screen",
        parameters=[
            {"arm_num": LaunchConfiguration("arm_num")},
            {"server_host_1": LaunchConfiguration("server_host_1")},
            {"robot_speed": LaunchConfiguration("robot_speed")},
            {"robot_acc": LaunchConfiguration("robot_acc")},
            {"trajectory_speed_scale": LaunchConfiguration("trajectory_speed_scale")},
            {"sync_with_trajectory_timing": LaunchConfiguration("sync_with_trajectory_timing")},
        ],
        condition=IfCondition(LaunchConfiguration("use_real_robot")),
    )

    duco_robot_status = Node(
        package="duco_ros_driver",
        executable="DucoRobotStatus",
        output="screen",
        parameters=[
            {"arm_num": LaunchConfiguration("arm_num")},
            {"arm_dof": LaunchConfiguration("arm_dof")},
            {"server_host_1": LaunchConfiguration("server_host_1")},
        ],
        remappings=[
            ("joint_states", "real_joint_states"),
            ("feedback_states", "real_feedback_states"),
        ],
        condition=IfCondition(LaunchConfiguration("use_real_robot")),
    )

    duco_robot_control = Node(
        package="duco_ros_driver",
        executable="DucoRobotControl",
        output="screen",
        parameters=[
            {"arm_num": LaunchConfiguration("arm_num")},
            {"arm_dof": LaunchConfiguration("arm_dof")},
            {"server_host_1": LaunchConfiguration("server_host_1")},
        ],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("use_real_robot"),
                "' == 'true' and '",
                LaunchConfiguration("use_real_control_services"),
                "' == 'true'",
            ])
        ),
    )

    moveit_params = moveit_config.to_dict()
    moveit_params.update(load_yaml_file(moveit_controllers_file))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, {"use_sim_time": True}],
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
            load_yaml_file(moveit_controllers_file),
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
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_real_robot", default_value="true"),
            DeclareLaunchArgument("use_real_control_services", default_value="true"),
            DeclareLaunchArgument("arm_num", default_value="1"),
            DeclareLaunchArgument("arm_dof", default_value="6"),
            DeclareLaunchArgument("server_host_1", default_value="192.168.4.23"),
            DeclareLaunchArgument("robot_speed", default_value="0.1"),
            DeclareLaunchArgument("robot_acc", default_value="0.15"),
            DeclareLaunchArgument("trajectory_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("sync_with_trajectory_timing", default_value="true"),
            DeclareLaunchArgument("sim_time_scale", default_value="1.0"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[duco_support_share_parent, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            gz_sim,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            start_sim_controllers,
            dual_trajectory_action,
            duco_driver,
            duco_robot_status,
            duco_robot_control,
            move_group,
            rviz,
        ]
    )
