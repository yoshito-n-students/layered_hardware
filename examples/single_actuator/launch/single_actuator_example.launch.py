from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Default argument values
    default_robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware"),
            "examples/single_actuator/description",
            "robot_description.urdf.xacro",
        ]
    )

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_path",
            default_value=default_robot_description_path,
            description="Path to robot URDF/xacro file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file",
        )
    )

    # Initialize Arguments
    robot_description_path = LaunchConfiguration("robot_description_path")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Get controller configs
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware"),
            "examples/single_actuator/config",
            "controllers.yaml",
        ]
    )

    # Get display configs
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware"), 
            "examples/single_actuator/config", 
            "display_config.rviz"]
    )

    # Declare control nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "position_controller",
                   "--controller-manager", "/controller_manager"],
    )
    sub_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "effort_controller",
                   "--inactive", "--controller-manager", "/controller_manager"],
    )

    # Declare visualization nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Build all nodes
    nodes = [
        control_node,
        robot_state_pub_node,
        controller_spawner,
        sub_controller_spawner,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)