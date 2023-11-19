from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation clock",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging severity level",
    )

    cfg_file_arg = DeclareLaunchArgument(
        "cfg_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("ros2_package"),
                "config",
                "ros2_package.yaml",
            ]
        ),
        description="The path of the ros2_package node configuration file",
    )

    ros2_package_node = Node(
        package="ros2_package",
        executable="ros2_package",
        namespace="namespace",
        parameters=[
            LaunchConfiguration("cfg_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    actions = [
        use_sim_time_arg,
        log_level_arg,
        cfg_file_arg,
        ros2_package_node,
    ]

    return LaunchDescription(actions)
