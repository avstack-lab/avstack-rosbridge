import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    track_config = os.path.join(
        get_package_share_directory("avstack_tracking"),
        "config",
        "boxtracker3d.yaml",
    )

    track_node = Node(
        package="avstack_tracking",
        executable="boxtracker3d",
        name="tracker",
        parameters=[track_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([track_node])
