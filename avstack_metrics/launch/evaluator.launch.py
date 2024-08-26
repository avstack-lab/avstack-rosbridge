import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    metrics_config = os.path.join(
        get_package_share_directory("avstack_metrics"),
        "config",
        "metrics.yaml",
    )

    metrics_node = Node(
        package="avstack_metrics",
        executable="evaluator",
        name="metrics",
        parameters=[metrics_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([metrics_node])