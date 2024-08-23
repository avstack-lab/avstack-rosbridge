from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    writer_config_launch_arg = DeclareLaunchArgument(
        "writer_config", default_value="cte_attack_1.yaml"
    )

    writer_config = PathJoinSubstitution(
        [
            get_package_share_directory("avstack_rosbag"),
            "config",
            LaunchConfiguration("writer_config"),
        ]
    )

    writer_node = Node(
        package="avstack_rosbag",
        executable="carla_dataset_writer_with_adversary",
        name="writer",
        parameters=[writer_config],
        arguments=["--ros-args", "--log-level", "INFO"],
        on_exit=Shutdown(),
    )

    return LaunchDescription([writer_config_launch_arg, writer_node])
