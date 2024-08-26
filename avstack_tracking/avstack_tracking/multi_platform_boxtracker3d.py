import rclpy
from avstack.datastructs import DataContainer
from avstack.modules.tracking.multisensor import MeasurementBasedMultiTracker
from avstack.modules.tracking.tracker3d import BasicBoxTracker3D
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from std_msgs.msg import Header, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray

from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge
from avstack_bridge.transform import do_transform_detection3d
from avstack_msgs.msg import BoxTrackArray


class MultiPlatformBoxTracker(Node):
    def __init__(self, verbose: bool = False):
        super().__init__("tracker")
        self.verbose = verbose
        self.declare_parameter("n_agents", 4)

        # initialize model
        self.model = MeasurementBasedMultiTracker(
            tracker=BasicBoxTracker3D(check_reference=False)
        )

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )

        # listen to transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to initialization message (optional)
        self.subscriber_init = self.create_subscription(
            String,
            "/initialization",
            self.init_callback,
            qos_profile=qos,
        )

        # subscribe to 3d tracks from agents
        self.subscriber_dets = {
            agent_ID: Subscriber(
                self,
                Detection3DArray,
                f"/agent{agent_ID}/detections_3d",
                qos_profile=qos,
            )
            for agent_ID in range(self.get_parameter("n_agents").value)
        }

        # synchronize messages from agents
        self.synchronizer_dets = ApproximateTimeSynchronizer(
            tuple(self.subscriber_dets.values()), queue_size=10, slop=0.1
        )
        self.synchronizer_dets.registerCallback(self.dets_receive)

        # publish tracks from command center
        self.publisher_trks = self.create_publisher(
            BoxTrackArray,
            "tracks_3d",
            qos_profile=qos,
        )

    def init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on box tracker!")
            self.model.reset()

    def dets_receive(self, *args):
        """Receive approximately synchronized detections

        Since we set a dynamic number of agents, we have to use star input
        """
        if self.verbose:
            self.get_logger().info(f"Received {len(args)} detection messages!")

        # convert all tracks to global reference frame
        dets_global = {}
        fovs = {}
        transforms = {}
        for dets_msg in args:
            agent = dets_msg.header.frame_id.split("/")[0]

            # HACK: no fov model for now
            fovs[agent] = None

            # get track info and convert to global
            dets_global[agent] = []
            try:
                tf_world_dets = self.tf_buffer.lookup_transform(
                    "world",
                    dets_msg.header.frame_id,
                    dets_msg.header.stamp,
                )
                transforms[agent] = tf_world_dets
                dets_global[agent] = DataContainer(
                    frame=0,
                    timestamp=Bridge.rostime_to_time(dets_msg.header.stamp),
                    data=[
                        DetectionBridge.detection_to_avstack(
                            do_transform_detection3d(boxdet, tf_world_dets),
                        )
                        for boxdet in dets_msg.detections
                    ],
                    source_identifier=agent,
                )
            except TransformException:
                self.get_logger().info(
                    f"Could not transform detections for multi-platform tracking"
                )
                return
            finally:
                last_stamp = dets_msg.header.stamp

        # perform tracking in global
        trks_avstack = self.model(dets_global, fovs=fovs, check_reference=False)
        header_out = Header(frame_id="world", stamp=last_stamp)
        trks_ros = TrackBridge.avstack_to_tracks(trks_avstack, header=header_out)
        self.publisher_trks.publish(trks_ros)

        if self.verbose:
            self.get_logger().info(f"Published {len(trks_avstack)} tracks!")


def main(args=None):
    rclpy.init(args=args)

    tracker = MultiPlatformBoxTracker()

    rclpy.spin(tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
