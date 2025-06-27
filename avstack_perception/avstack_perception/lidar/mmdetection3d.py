import rclpy
from avstack.modules.perception.object3d import MMDetObjectDetector3D
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as LidarMsg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray

from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.sensors import LidarSensorBridge


class LidarPerception(Node):
    def __init__(self, verbose: bool = False):
        super().__init__("perception")
        self.declare_parameter("perception_model", "pointpillars")
        self.declare_parameter("perception_dataset", "carla-vehicle")

        # initialize models
        param_model = self.get_parameter("perception_model").value
        param_dataset = self.get_parameter("perception_dataset").value
        self.model = MMDetObjectDetector3D(
            model=param_model,
            dataset=param_dataset,
        )
        self.get_logger().info(
            f"Initialized {param_model} model on {param_dataset} dataset (MMDet3D)"
        )
        self.verbose = verbose

        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # listen to transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to point cloud in the same namespace
        self.subscriber_pc = self.create_subscription(
            LidarMsg, "point_cloud", self.pc_callback, qos_profile=qos_profile
        )

        # publish lidar detections
        self.publisher_dets = self.create_publisher(
            Detection3DArray,
            "detections_3d",
            qos_profile=qos_profile,
        )

    def pc_callback(self, pc_msg: LidarMsg):
        """Run perception model when we receive lidar data

        We need to obtain the actual reference frame for the lidar sensor
        because the perception model runs detection on a nominal sensor
        height. Therefore, we need to adjust the perception outcomes
        based on the difference between the nominal height and the realized
        sensor height.
        """
        # get the transformations
        # HACK: assume the sensor is e.g., agent0/lidar0
        try:
            frame_agent = pc_msg.header.frame_id.split("/")[0]
            # transform that takes **points** from source=agent to target=world
            tf_world_agent = self.tf_buffer.lookup_transform(
                target_frame="world",
                source_frame=frame_agent,
                time=pc_msg.header.stamp,
            )
            # transform that takes **points** from source=lidar to target=agent
            tf_agent_lidar = self.tf_buffer.lookup_transform(
                target_frame=frame_agent,
                source_frame=pc_msg.header.frame_id,
                time=pc_msg.header.stamp,
            )
            ref_agent_world = Bridge.tf2_to_reference(tf_world_agent)
            ref_lidar_agent = Bridge.tf2_to_reference(
                tf_agent_lidar, reference=ref_agent_world
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform point cloud for perception")
            return

        # run the pipeline
        pc_avstack = LidarSensorBridge.pc2_to_avstack(pc_msg)
        pc_avstack.calibration.reference = ref_lidar_agent
        dets_avstack = self.model(pc_avstack)
        if self.verbose:
            self.get_logger().info(
                f"Lidar perception generated {len(dets_avstack)} detections"
            )
        dets_ros = DetectionBridge.avstack_to_detectionarray(
            dets_avstack, header=pc_msg.header
        )
        self.publisher_dets.publish(dets_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = LidarPerception()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
