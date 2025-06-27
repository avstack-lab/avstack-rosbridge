import numpy as np
import rclpy
from avstack.modules.tracking.tracker2d import BasicBoxTracker2D
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrack2DArray


class BoxTracker2D(Node):
    def __init__(self):
        super().__init__("tracker")

        # initialize model
        self.model = BasicBoxTracker2D(
            threshold_coast=3, threshold_confirmed=3, check_reference=False
        )
        self.get_logger().info("Initialized BasicBoxTracker2D")

        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # subscribe to initialization message (optional)
        self.subscriber_init = self.create_subscription(
            String,
            "/initialization",
            self.init_callback,
            qos_profile=qos_profile,
        )

        # subscribe to 2d detections
        self.subscriber_dets = self.create_subscription(
            Detection2DArray,
            "detections_2d",
            self.dets_callback,
            qos_profile=qos_profile,
        )

        # publish 2d tracks
        self.publisher_trks = self.create_publisher(
            BoxTrack2DArray,
            "tracks_2d",
            qos_profile=qos_profile,
        )

    def init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on box tracker!")
            self.model.reset()

    def dets_callback(self, dets_msg: Detection2DArray):
        # perform tracking
        dets_avstack = DetectionBridge.detectionarray_to_avstack(
            dets_msg=dets_msg,
            noise=np.array([10, 10, 10, 10]) ** 2,  # TODO: pull this out
        )
        platform = Bridge.header_to_reference(dets_msg.header)
        trks_avstack = self.model(
            dets_avstack, platform=platform, check_reference=False
        )
        trks_ros = TrackBridge.avstack_to_tracks(
            trks_avstack, is_2d=True, header=dets_msg.header
        )
        trks_ros_local = trks_ros
        self.publisher_trks.publish(trks_ros_local)


def main(args=None):
    rclpy.init(args=args)

    tracker = BoxTracker2D()

    rclpy.spin(tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
