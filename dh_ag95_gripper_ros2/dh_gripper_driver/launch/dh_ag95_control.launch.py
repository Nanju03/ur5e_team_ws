import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def configure_force(context):
    value = float(LaunchConfiguration("force_percent").perform(context))
    if not math.isfinite(value) or not value.is_integer() or not 20 <= value <= 100:
        raise ValueError("force_percent must be an integer from 20 through 100")
    return [SetLaunchConfiguration("gripper_force_multiplier", str((value - 20) / 80))]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="gripper",
            description="ROS namespace separating the gripper from the arm controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/robot/dh_ag95_gripper",
            description="COM port of the gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "force_percent", default_value="20",
            description=(
                "Initial DH force setting, integer 20..100 percent; "
                "not a measured force in newtons."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mocked command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_robot_state_publisher",
            default_value="true",
            description=(
                "Publish the standalone gripper model. "
                "Disable when a combined robot model publishes TF."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="joint_states",
            description=(
                "Joint feedback topic; use /joint_states to share feedback "
                "with a combined arm model."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_mode",
            default_value="forward",
            choices=["forward", "action"],
            description="Use direct position commands or the MoveIt gripper action controller.",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    com_port = LaunchConfiguration("com_port")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    gui = LaunchConfiguration("gui")
    launch_robot_state_publisher = LaunchConfiguration("launch_robot_state_publisher")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dh_ag95_description"),
                    "urdf",
                    "dh_ag95.urdf.xacro",
                ]
            ),
            " ",
            "com_port:=",
            com_port,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "gripper_force_multiplier:=",
            LaunchConfiguration("gripper_force_multiplier"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Load controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dh_gripper_driver"),
            "config",
            "dh_ag95_controllers.yaml",
        ]
    )
    # Load RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dh_gripper_driver"), "rviz", "dh_ag95_control.rviz"]
    )

    # controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            # The broadcaster publishes a relative topic in its /gripper namespace.
            # Match that name as well as versions using the absolute topic.
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
            ("dynamic_joint_states", "dynamic_joint_states"),
            ("/dynamic_joint_states", "dynamic_joint_states"),
        ],
    )
    # robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=IfCondition(launch_robot_state_publisher),
        namespace=namespace,
        output="both",
        parameters=[robot_description],
        # This standalone model is rooted at world, not attached to the UR wrist.
        # Keep its TF tree separate until a combined robot description is used.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    # rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        remappings=[
            ("/robot_description", "robot_description"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # helper function to spawn controllers
    def controller_spawner(name, *args, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            condition=condition,
            output="screen",
            # A relative manager name resolves inside this spawner's namespace.
            arguments=[name, "--controller-manager", "controller_manager"] + list(args),
        )

    # Keep the original forward controller behavior unless action mode is selected.
    active_spawners = [
        controller_spawner("joint_state_broadcaster"),
        controller_spawner("force_forward_controller"),
        controller_spawner(
            "forward_position_controller",
            condition=LaunchConfigurationEquals("controller_mode", "forward"),
        ),
        controller_spawner(
            "dh_ag95_gripper_controller",
            condition=LaunchConfigurationEquals("controller_mode", "action"),
        ),
    ]
    # Both controllers command the same joint, so forward stays inactive in action mode.
    inactive_spawners = [
        controller_spawner(
            "forward_position_controller",
            "--inactive",
            condition=LaunchConfigurationEquals("controller_mode", "action"),
        ),
    ]

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
    ] + active_spawners + inactive_spawners

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=configure_force)] + nodes
    )
