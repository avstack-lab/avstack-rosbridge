import rclpy
from avstack.modules.perception.object2dfv import MMDetObjectDetector2D
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection2DArray

from avstack_bridge.detections import DetectionBridge
from avstack_bridge.sensors import CameraSensorBridge


class CameraPerception(Node):
    def __init__(self, verbose: bool = False):
        super().__init__("perception")
        self.declare_parameter("perception_model", "rtmdet")
        self.declare_parameter("perception_dataset", "rccars-oneclass")

        # initialize models
        param_model = self.get_parameter("perception_model").value
        param_dataset = self.get_parameter("perception_dataset").value
        self.model = MMDetObjectDetector2D(
            model=param_model,
            dataset=param_dataset,
            epoch="latest",
            threshold=0.7,
            gpu=0,
        )
        self.get_logger().info(
            f"Initialized {param_model} model on {param_dataset} dataset (MMDet2D)"
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
        self.subscriber_img = self.create_subscription(
            ImageMsg, "image", self.img_callback, qos_profile=qos_profile
        )

        # publish lidar detections
        self.publisher_dets = self.create_publisher(
            Detection2DArray,
            "detections_2d",
            qos_profile=qos_profile,
        )

    def img_callback(self, img_msg: ImageMsg):
        """Run perception model when we receive camera data"""

        # run the pipeline
        img_avstack = CameraSensorBridge.imgmsg_to_avstack(img_msg)
        dets_avstack = self.model(img_avstack)
        if self.verbose:
            self.get_logger().info(
                f"Camera perception generated {len(dets_avstack)} detections"
            )
        dets_ros = DetectionBridge.avstack_to_detectionarray(
            dets_avstack, det_is_2d=True, header=img_msg.header
        )
        self.publisher_dets.publish(dets_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = CameraPerception()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
