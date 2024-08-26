import rclpy
from avstack.modules.perception.fov_estimator import ConcaveHullLidarFOVEstimator
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as LidarMsg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from avstack_bridge.geometry import GeometryBridge
from avstack_bridge.sensors import LidarSensorBridge
from avstack_bridge.transform import do_transform_cloud


# from tf2_geometry_msgs import do_transform_polygon_stamped


class LidarFovEstimator(Node):
    def __init__(self):
        super().__init__("fov_estimator")
        self.model = ConcaveHullLidarFOVEstimator(max_height=5)

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )

        # listen to transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to point cloud in the same namespace
        self.subscriber_pc = self.create_subscription(
            LidarMsg, "point_cloud", self.pc_callback, qos_profile=qos
        )

        # publish fov model as point cloud
        self.publisher_fov = self.create_publisher(
            PolygonStamped,
            "fov",
            qos_profile=qos,
        )

    def pc_callback(self, pc_msg: LidarMsg) -> LidarMsg:
        """Run the FOV estimation model when we receive lidar data

        We need to project the lidar data into either a ground or global
        reference frame to handle perspective angle sensors
        """
        try:
            # transform that takes **points** from source=lidar to target=world
            tf_world_lidar = self.tf_buffer.lookup_transform(
                target_frame="world",
                source_frame=pc_msg.header.frame_id,
                time=pc_msg.header.stamp,
            )
        except TransformException:
            self.get_logger().info(
                f"Could not transform point cloud for fov estimation"
            )
            return
        pc_msg_global = do_transform_cloud(pc_msg, tf_world_lidar)
        pc_avstack = LidarSensorBridge.pc2_to_avstack(pc_msg_global)
        fov_avstack = self.model(pc_avstack)
        fov_ros = GeometryBridge.avstack_to_polygon(fov_avstack, stamped=True)
        self.publisher_fov.publish(fov_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = LidarFovEstimator()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
