import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
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


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def validate_real_robot_settings(context, *args, **kwargs):
    """Fail loudly instead of silently launching a different configuration."""
    use_real = LaunchConfiguration("use_real_robot").perform(context)
    wait_for_real = LaunchConfiguration("wait_for_real").perform(context)
    host = LaunchConfiguration("server_host_1").perform(context)

    if _is_true(use_real) and not str(host).strip():
        raise RuntimeError(
            "use_real_robot:=true requires an explicit server_host_1. The previous hard-coded "
            "address 192.168.6.4 was removed because a historical address is not evidence that it "
            "is the robot's current address."
        )
    if _is_true(wait_for_real) and not _is_true(use_real):
        raise RuntimeError(
            "wait_for_real:=true requires use_real_robot:=true; there is nothing to wait for when "
            "the real channel is disabled."
        )
    return []


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

    start_sim_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, sim_arm_controller_spawner],
    )

    dual_trajectory_action = Node(
        package="duco_ros_driver",
        executable="DualTrajectoryAction",
        output="screen",
        parameters=[
            {"front_action_name": "arm_1_controller/follow_joint_trajectory"},
            {"sim_action_name": "sim_arm_1_controller/follow_joint_trajectory"},
            {"real_command_topic": "joint_path_command"},
            {"publish_real": LaunchConfiguration("use_real_robot")},
            {"wait_for_real": LaunchConfiguration("wait_for_real")},
            {"real_feedback_topic": LaunchConfiguration("real_feedback_topic")},
            {"real_status_topic": LaunchConfiguration("real_status_topic")},
            {"real_goal_tolerance": ParameterValue(
                LaunchConfiguration("real_goal_tolerance"), value_type=float)},
            {"real_velocity_tolerance": ParameterValue(
                LaunchConfiguration("real_velocity_tolerance"), value_type=float)},
            {"real_feedback_max_age": ParameterValue(
                LaunchConfiguration("real_feedback_max_age"), value_type=float)},
            {"real_status_max_age": ParameterValue(
                LaunchConfiguration("real_status_max_age"), value_type=float)},
            {"real_require_velocity": ParameterValue(
                LaunchConfiguration("real_require_velocity"), value_type=bool)},
            {"real_motion_state_topic": LaunchConfiguration("real_motion_state_topic")},
            {"min_real_trajectory_points": ParameterValue(
                LaunchConfiguration("min_real_trajectory_points"), value_type=int)},
            {"clear_fault_service": LaunchConfiguration("clear_fault_service")},
            {"fault_recovery_enabled": ParameterValue(
                LaunchConfiguration("fault_recovery_enabled"), value_type=bool)},
            {"latch_on_real_fault": ParameterValue(
                LaunchConfiguration("latch_on_real_fault"), value_type=bool)},
            {"fault_latch_file": LaunchConfiguration("fault_latch_file")},
            {"real_persistence_required": ParameterValue(
                LaunchConfiguration("real_persistence_required"), value_type=bool)},
            {"real_execution_timeout": ParameterValue(
                LaunchConfiguration("real_execution_timeout"), value_type=float)},
            {"publish_real_stop_request": ParameterValue(
                LaunchConfiguration("publish_real_stop_request"), value_type=bool)},
            {"acceptance_timeout": ParameterValue(
                LaunchConfiguration("acceptance_timeout"), value_type=float)},
            {"sim_execution_timeout": ParameterValue(
                LaunchConfiguration("sim_execution_timeout"), value_type=float)},
            {"cancel_timeout": ParameterValue(
                LaunchConfiguration("cancel_timeout"), value_type=float)},
            {"enforce_expected_joint_names": ParameterValue(
                LaunchConfiguration("enforce_expected_joint_names"), value_type=bool)},
            {"sim_time_scale": ParameterValue(
                LaunchConfiguration("sim_time_scale"), value_type=float)},
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
        parameters=[
            moveit_params,
            {"use_sim_time": True},
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0},
            {"trajectory_execution.allowed_goal_duration_margin": 5.0},
        ],
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
            # The real channel must be enabled explicitly: a plain launch must
            # never be able to command the physical arm.
            DeclareLaunchArgument("use_real_robot", default_value="false"),
            DeclareLaunchArgument("use_real_control_services", default_value="true"),
            DeclareLaunchArgument("arm_num", default_value="1"),
            DeclareLaunchArgument("arm_dof", default_value="6"),
            # No address is assumed any more.  A historical IP is not evidence
            # that it is the robot's current address, so use_real_robot:=true
            # requires an explicit server_host_1.
            DeclareLaunchArgument("server_host_1", default_value=""),
            DeclareLaunchArgument("robot_speed", default_value="0.1"),
            DeclareLaunchArgument("robot_acc", default_value="0.15"),
            DeclareLaunchArgument("trajectory_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("sync_with_trajectory_timing", default_value="true"),
            DeclareLaunchArgument("sim_time_scale", default_value="1.0"),
            # Only meaningful together with use_real_robot:=true.  Defaulting it
            # to false keeps the default launch a pure simulation, and asking for
            # a closed-loop dual run is then an explicit choice.
            DeclareLaunchArgument("wait_for_real", default_value="false"),
            DeclareLaunchArgument("real_feedback_topic", default_value="real_feedback_states"),
            DeclareLaunchArgument("real_status_topic", default_value="/duco_cobot/robot_state"),
            DeclareLaunchArgument("real_goal_tolerance", default_value="0.02"),
            DeclareLaunchArgument("real_velocity_tolerance", default_value="0.05"),
            DeclareLaunchArgument("real_feedback_max_age", default_value="1.0"),
            DeclareLaunchArgument("real_status_max_age", default_value="1.0"),
            DeclareLaunchArgument("real_require_velocity", default_value="true"),
            DeclareLaunchArgument(
                "real_motion_state_topic", default_value="dual_trajectory/real_motion_state"
            ),
            DeclareLaunchArgument("min_real_trajectory_points", default_value="2"),
            DeclareLaunchArgument("clear_fault_service", default_value="clear_fault"),
            # Enabling the recovery service is an explicit operator action; a
            # ROS 2 service cannot be authenticated on its own.
            DeclareLaunchArgument("fault_recovery_enabled", default_value="false"),
            DeclareLaunchArgument("latch_on_real_fault", default_value="true"),
            # Empty selects the node's built-in default, which is a per-user
            # state directory ($XDG_STATE_HOME or ~/.local/state).  A shared
            # temporary directory is deliberately not used: it is emptied by tmp
            # cleaners and by reboots, which would erase the memory of an
            # unconfirmed real motion.
            DeclareLaunchArgument("fault_latch_file", default_value=""),
            # When true the interlock state file must be writable before a real
            # trajectory may be published; otherwise real motion is refused.
            DeclareLaunchArgument("real_persistence_required", default_value="true"),
            DeclareLaunchArgument("real_execution_timeout", default_value="300.0"),
            DeclareLaunchArgument("publish_real_stop_request", default_value="true"),
            DeclareLaunchArgument("acceptance_timeout", default_value="5.0"),
            DeclareLaunchArgument("sim_execution_timeout", default_value="300.0"),
            DeclareLaunchArgument("cancel_timeout", default_value="5.0"),
            DeclareLaunchArgument("enforce_expected_joint_names", default_value="true"),
            OpaqueFunction(function=validate_real_robot_settings),
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
