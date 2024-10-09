from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    #declared_arguments.append(
    #    DeclareLaunchArgument(
    #        "gui",
    #        default_value="true",
    #        description="Start RViz2 automatically with this launch file.",
    #    )
    #)

    # Initialize Arguments
    # gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("layered_hardware"),
                    "urdf",
                    "example.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware"),
            "config",
            "example_controllers.yaml",
        ]
    )
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_description"), "rrbot/rviz", "rrbot.rviz"]
    # )

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
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "shoulder_position_controller", "elbow_position_controller", "wrist_position_controller", 
                   "--controller-manager", "/controller_manager"],
    )

    sub_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["shoulder_velocity_controller", "elbow_velocity_controller", "wrist_velocity_controller", 
                   "--inactive", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        controller_spawner,
        sub_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)