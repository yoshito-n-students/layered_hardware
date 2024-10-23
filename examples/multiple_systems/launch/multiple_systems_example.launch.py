from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # first system in "system1" namespace
    system1_nodes = GroupAction(
        actions=[
            PushRosNamespace("system1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("layered_hardware"),
                        "examples/single_actuator/launch",
                        "single_actuator_example.launch.py"])
                ]),
                launch_arguments={
                    "gui": "false"
                }.items()
            )
        ]
    )

    # second system in "system2" namespace
    system2_nodes = GroupAction(
        actions=[
            PushRosNamespace("system2"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("layered_hardware"),
                        "examples/single_actuator/launch",
                        "single_actuator_example.launch.py"])
                ]),
                launch_arguments={
                    "gui": "false"
                }.items()
            )
        ]
    )

    return LaunchDescription([
        system1_nodes, system2_nodes

    ])
