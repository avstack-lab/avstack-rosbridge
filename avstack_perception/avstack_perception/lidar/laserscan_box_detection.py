from typing import List

import numpy as np
import rclpy
from avstack.datastructs import DataContainer
from avstack.geometry import Attitude, Box3D, Position
from avstack.modules.perception.detections import BoxDetection
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from vision_msgs.msg import Detection3DArray

from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge


class Cluster:
    def __init__(self, points: np.ndarray):
        """Points is N x D where N is num points and D is dimension"""
        self.points = points

        # compute stats on the clusters
        self.centroid = np.mean(points, axis=0)
        self.ranges = np.linalg.norm(points, axis=1)
        self.azimuths = np.arctan2(points[:, 1], points[:, 0])

    @property
    def dim(self):
        return len(self.centroid)

    @property
    def n_points(self):
        return len(self.points)

    def center(self) -> np.ndarray:
        """Return the centroid (mean) of the cluster as a 2D point."""
        return np.mean(self.points, axis=0)

    def size(self) -> int:
        return self.points.shape[0]

    def bounding_box(self) -> np.ndarray:
        x_min, y_min = np.min(self.points, axis=0)
        x_max, y_max = np.max(self.points, axis=0)
        return np.array([[x_min, y_min], [x_max, y_max]])


def laserscan_to_points(scan_data: LaserScan):
    """Convert laser scan to numpy array of points"""
    ranges = np.array(scan_data.ranges)
    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))

    # Handle infinite values (replace with maximum range if needed)
    ranges[np.isinf(ranges)] = scan_data.range_max

    # Calculate x and y coordinates
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Stack x and y coordinates to create points array
    points = np.stack((x, y), axis=-1)
    return points


class ObjectDetectionClusterer:
    """Runs a clustering algorithm to detect obstacles

    obstacles will be dense collections of points
    """

    def __init__(self):
        self.clusterer = DBSCAN(
            eps=0.2,
            min_samples=5,
            metric="euclidean",
            algorithm="auto",
        )

    def __call__(self, points: np.ndarray) -> List[Cluster]:

        # run clustering algorithm
        db = self.clusterer.fit(points)

        # filter the points in the clusters
        clusters = [
            Cluster(points[db.labels_ == k, :])
            for k in set(db.labels_)
            if k != -1  # -1 is just noise
        ]
        return clusters


class LaserScanBoxDetection(Node):
    def __init__(self, verbose: bool = False):
        super().__init__("perception")

        # initialize perception model
        self.model = ObjectDetectionClusterer()
        self.get_logger().info("Initialized DBSCAN clusterer")

        # qos profile for pub/sub
        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # subscribe to lidar laser scan
        self.subscriber_lidar = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            qos_profile=qos_profile,
        )

        # publisher for box detections
        self.publisher_dets = self.create_publisher(
            Detection3DArray,
            "detections_3d",
            qos_profile=qos_profile,
        )

    def _cluster_to_3d_box_detection(
        self, cluster: Cluster, platform, height: float = 0.5
    ) -> BoxDetection:
        # build 3d box
        width = np.max(cluster.points[:, 0]) - np.min(cluster.points[:, 0])
        length = np.max(cluster.points[:, 1]) - np.min(cluster.points[:, 1])
        box = Box3D(
            position=Position(
                x=np.array(
                    [
                        cluster.centroid[0],
                        cluster.centroid[1],
                        0,
                    ]
                ),
                reference=platform,
            ),
            attitude=Attitude(q=np.quaternion(1), reference=platform),
            hwl=[height, width, length],  # TODO set this more intelligently
            where_is_t="center",
            obj_type="car",
        )

        # format as detection
        detection = BoxDetection(
            data=box,
            noise=np.array([1, 1, 1, 0.25, 0.25, 0.25]) ** 2,
            source_identifier="0",
            reference=box.reference,
            obj_type=box.obj_type,
            score=1.0,
        )

        return detection

    def lidar_callback(self, msg: LaserScan):
        # Convert LaserScan to 2D points
        points = laserscan_to_points(msg)

        # Detect clusters (objects)
        clusters = self.model(points)

        # Extend to 3D boxes with approximate shapes
        platform = Bridge.header_to_reference(msg.header)
        boxes_3d = DataContainer(
            frame=0,
            timestamp=Bridge.rostime_to_time(msg.header.stamp),
            data=[
                self._cluster_to_3d_box_detection(cluster, platform)
                for cluster in clusters
            ],
            source_identifier="0",
        )

        # convert to ros types and publish
        boxes_3d_ros = DetectionBridge.avstack_to_detectionarray(
            boxes_3d, header=msg.header
        )
        self.publisher_dets.publish(boxes_3d_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = LaserScanBoxDetection()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
