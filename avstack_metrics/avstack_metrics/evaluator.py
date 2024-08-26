import numpy as np
import rclpy
from avapi.evaluation.metrics.ospa import OspaMetric
from avstack.modules.assignment import gnn_single_frame_assign
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrackArray, ObjectStateArray, TrackingMetrics
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node


class MetricsEvalator(Node):
    def __init__(self):
        super().__init__("metrics")

        # subscribe to tracks and truths and synch them
        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )

        # tracks topic
        self.subscriber_tracks = Subscriber(
            self, BoxTrackArray, f"/tracks", qos_profile=qos
        )

        # truths topic
        self.subscriber_truths = Subscriber(
            self, ObjectStateArray, f"/truths", qos_profile=qos
        )

        # synchronize
        self.synchronizer_tracks = ApproximateTimeSynchronizer(
            (self.subscriber_tracks, self.subscriber_truths), queue_size=10, slop=0.1
        )
        self.synchronizer_tracks.registerCallback(self.receive)

        # publish metrics list
        self.publisher_metrics = self.create_publisher(
            msg_type=TrackingMetrics, topic="tracking_metrics", qos_profile=qos
        )

    def receive(
        self,
        msg_tracks: BoxTrackArray,
        msg_truths: ObjectStateArray,
        assign_radius: float = 2.0,
    ):
        header = msg_truths.header

        # convert to avstack types
        _tracks = TrackBridge.tracks_to_avstack(msg_tracks)
        _truths = ObjectStateBridge.objectstatearray_to_avstack(msg_truths)
        trks_position = [track.position.x for track in _tracks]
        trus_position = [
            truth.position.x for truth in _truths if "static" not in truth.obj_type
        ]

        # compute metrics
        A = np.array(
            [[np.linalg.norm(l1 - l2) for l1 in trks_position] for l2 in trus_position]
        )
        assign = gnn_single_frame_assign(A, cost_threshold=assign_radius)
        ospa = OspaMetric.cost(trks_position, trus_position)

        # publish output
        metrics_msg = TrackingMetrics(
            header=header,
            n_truths=len(trus_position),
            n_tracks=len(trks_position),
            n_assign=len(assign),
            ospa=ospa,
        )
        self.publisher_metrics.publish(metrics_msg)


def main(args=None):
    rclpy.init(args=args)

    metrics = MetricsEvalator()

    rclpy.spin(metrics)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    metrics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
