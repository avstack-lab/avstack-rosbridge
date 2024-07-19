import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    replayer_config = os.path.join(
        get_package_share_directory("avstack_rosbag"),
        "config",
        "carla_dataset_recorder.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="avstack_rosbag",
                executable="carla_dataset_recorder",
                name="recorder",
                parameters=[replayer_config],
                arguments=["--ros-args", "--log-level", "INFO"],
                on_exit=Shutdown(),
            ),
        ]
    )
