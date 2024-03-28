from avstack.datastructs import DataContainer
from geometry_msgs.msg import PoseWithCovariance, Pose as gPose, Point
from vision_msgs.msg import Pose2D, Point2D, ObjectHypothesisWithPose, ObjectHypothesis
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, BoundingBox3DArray

from .base import Bridge
from .geometry import GeometryBridge


class DetectionBridge:
    @staticmethod
    def detections_2d_to_avstack(dets_msg: Detection2DArray) -> DataContainer:
        raise

    @staticmethod
    def avstack_to_detections_2d(dets: DataContainer, header=None) -> Detection2DArray:
        if header is None:
            raise
        box_dets = []
        source_image = ""
        for i, det in enumerate(dets):
            size_x = float(det.box.width)
            size_y = float(det.box.height)
            x = float(det.box.center[0])
            y = float(det.box.center[1])
            theta = 0.0
            position = Point2D(x=x, y=y)
            center = Pose2D(position=position, theta=theta)
            c_g = gPose(position=Point(x=x, y=y))
            c_with_cov = PoseWithCovariance(pose=c_g, covariance=36*[0])
            hypothesis = ObjectHypothesis(class_id="person", score=1.0)
            results = [ObjectHypothesisWithPose(hypothesis=hypothesis, pose=c_with_cov)]
            bbox = BoundingBox2D(center=center, size_x=size_x, size_y=size_y)
            box_det = Detection2D(header=header, results=results, bbox=bbox, id=source_image)
            box_dets.append(box_det)
        return Detection2DArray(header=header, detections=box_dets)


    @staticmethod
    def detections_3d_to_avstack(dets_msg: BoundingBox3DArray) -> DataContainer:
        timestamp = Bridge.rostime_to_time(dets_msg.header.stamp)
        dets = GeometryBridge.box3d_array_to_avstack(dets_msg)
        return DataContainer(
            frame=0, timestamp=timestamp, data=dets, source_identifier="0"
        )

    @staticmethod
    def avstack_to_detections_3d(dets: DataContainer, header=None) -> BoundingBox3DArray:
        return GeometryBridge.avstack_to_box3d_array(dets, header=header)
