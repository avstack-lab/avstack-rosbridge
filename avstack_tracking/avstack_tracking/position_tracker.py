import rclpy
from avstack.geometry import GlobalOrigin2D
from avstack.modules.tracking.tracker3d import BasicXyzTracker
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import XyzTrackArray


class ClusterTracker(Node):
    def __init__(self):
        super().__init__("cluster_tracker")

        # initialize tracker
        self.model = BasicXyzTracker(
            threshold_coast=8,
            threshold_confirmed=5,
            v_max=60,  # meters per second
            assign_metric="center_dist",
            assign_radius=3,
        )

        # qos profile for pub/sub
        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # # listen to transform information
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to initialization message (optional)
        self.subscriber_init = self.create_subscription(
            String,
            "/initialization",
            self.init_callback,
            qos_profile=qos_profile,
        )

        # subscribe to cluster center
        self.subscriber_dets = self.create_subscription(
            Detection2DArray,
            "clusters_lidar",
            self.dets_callback,
            qos_profile=qos_profile,
        )

        # publish 3d tracks
        self.publisher_trks = self.create_publisher(
            XyzTrackArray,
            "tracks_lidar",
            qos_profile=qos_profile,
        )

    def init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on box tracker!")
            self.model.reset()

    def dets_callback(self, dets_msg: Detection3DArray) -> XyzTrackArray:
        # perform reference conversion if needed
        dets_avstack = DetectionBridge.detectionarray_to_avstack(dets_msg)
        # platform = Bridge.header_to_reference(dets_msg.header)
        platform = GlobalOrigin2D
        trks_avstack = self.model(
            dets_avstack, platform=platform, check_reference=False
        )
        trks_ros = TrackBridge.avstack_to_tracks(trks_avstack, header=dets_msg.header)
        trks_ros_local = trks_ros
        self.publisher_trks.publish(trks_ros_local)


def main(args=None):
    rclpy.init(args=args)

    tracker = ClusterTracker()

    rclpy.spin(tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
