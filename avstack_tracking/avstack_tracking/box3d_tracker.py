import numpy as np
import rclpy
from avstack.modules.tracking.tracker3d import BasicBoxTracker3D
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray

from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge
from avstack_bridge.transform import do_transform_detection3d
from avstack_msgs.msg import BoxTrackArray


class BoxTracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.declare_parameter("tracking_in_global", False)

        # initialize model
        self.tracking_in_global = self.get_parameter("tracking_in_global").value
        self.model = BasicBoxTracker3D(check_reference=False)
        self.get_logger().info("Initialized BasicBoxTracker3D")

        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # listen to transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to initialization message (optional)
        self.subscriber_init = self.create_subscription(
            String,
            "/initialization",
            self.init_callback,
            qos_profile=qos_profile,
        )

        # subscribe to 3d detections
        self.subscriber_dets = self.create_subscription(
            Detection3DArray,
            "detections_3d",
            self.dets_callback,
            qos_profile=qos_profile,
        )

        # publish 3d tracks
        self.publisher_trks = self.create_publisher(
            BoxTrackArray,
            "tracks_3d",
            qos_profile=qos_profile,
        )

    def init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on box tracker!")
            self.model.reset()

    def dets_callback(self, dets_msg: Detection3DArray):
        # perform reference conversion if needed
        if self.tracking_in_global:
            try:
                # transform that takes **points** from source=agent to target=world
                tf_world_dets = self.tf_buffer.lookup_transform(
                    target_frame="world",
                    source_frame=dets_msg.header.frame_id,
                    time=dets_msg.header.stamp,
                )
                dets_msg_tf = Detection3DArray()
                dets_msg_tf.detections = [
                    do_transform_detection3d(det, tf_world_dets)
                    for det in dets_msg.detections
                ]
                dets_msg_tf.header = tf_world_dets.header
            except TransformException:
                self.get_logger().info(f"Could not transform detections for tracking")
                return
        else:
            dets_msg_tf = dets_msg

        # perform tracking
        dets_avstack = DetectionBridge.detectionarray_to_avstack(
            dets_msg=dets_msg_tf,
            noise=np.array([1, 1, 1, 0.25, 0.25, 0.25]) ** 2,  # TODO: pull this out
        )
        platform = Bridge.header_to_reference(dets_msg_tf.header)
        trks_avstack = self.model(
            dets_avstack, platform=platform, check_reference=False
        )
        trks_ros = TrackBridge.avstack_to_tracks(
            trks_avstack, header=dets_msg_tf.header
        )
        trks_ros_local = trks_ros
        self.publisher_trks.publish(trks_ros_local)


def main(args=None):
    rclpy.init(args=args)

    tracker = BoxTracker()

    rclpy.spin(tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
