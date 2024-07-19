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
        "carla_dataset_replayer.yml",
    )

    output_folder = LaunchConfiguration("output_folder")

    return LaunchDescription(
        [
            Node(
                package="avstack_rosbag",
                executable="carla_dataset_replayer",
                name="replayer",
                parameters=[
                    replayer_config,
                    {"output_folder": output_folder},
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
                on_exit=Shutdown(),
            ),
        ]
    )
