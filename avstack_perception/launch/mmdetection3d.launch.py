import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    percep_config = os.path.join(
        get_package_share_directory("avstack_perception"),
        "config",
        "mmdetection3d.yaml",
    )

    percep_node = Node(
        package="avstack_perception",
        executable="mmdetection3d",
        name="perception",
        parameters=[percep_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([percep_node])
