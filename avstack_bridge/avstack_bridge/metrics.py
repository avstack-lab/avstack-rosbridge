from avstack.metrics.assignment import SingleFrameMetrics
from std_msgs.msg import Header

from avstack_bridge import Bridge
from avstack_msgs.msg import AssignmentMetrics as AssignmentMetricsRos


class MetricsBridge:
    """Conversions for metrics"""

    ###################################################
    # ROS --> AVstack methods
    ###################################################

    @staticmethod
    def assignment_metrics_ros_to_avstack(
        metrics: AssignmentMetricsRos,
    ) -> SingleFrameMetrics:
        metrics_avstack = SingleFrameMetrics(
            n_truths=metrics.n_truths,
            n_tracks=metrics.n_tracks,
            n_assign=metrics.n_assign,
            precision=metrics.precision,
            recall=metrics.recall,
            ospa=metrics.ospa,
            timestamp=Bridge.rostime_to_time(metrics.header.stamp),
        )
        return metrics_avstack

    @staticmethod
    def fov_metrics_ros_to_avstack():
        raise NotImplementedError

    ###################################################
    # AVstack --> ROS methods
    ###################################################

    @staticmethod
    def assignment_metrics_avstack_to_ros(
        metrics: SingleFrameMetrics,
    ) -> AssignmentMetricsRos:
        metrics_ros = AssignmentMetricsRos(
            header=Header(
                frame_id="world", stamp=Bridge.time_to_rostime(metrics.timestamp)
            ),
            n_truths=metrics.n_truths,
            n_tracks=metrics.n_tracks,
            n_assign=metrics.n_assign,
            precision=metrics.precision,
            recall=metrics.recall,
            ospa=metrics.ospa,
        )
        return metrics_ros

    @staticmethod
    def fov_metrics_avstack_to_ros():
        raise NotImplementedError
