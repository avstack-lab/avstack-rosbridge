import numpy as np
from avstack.datastructs import DataContainer
from avstack.modules.perception.detections import BoxDetection
from vision_msgs.msg import Detection3D as RosDetection3D
from vision_msgs.msg import Detection3DArray as RosDetection3DArray
from vision_msgs.msg import ObjectHypothesis, ObjectHypothesisWithPose

from .base import Bridge
from .geometry import GeometryBridge


class DetectionBridge:

    ##########################################
    # ROS --> AVstack
    ##########################################

    @staticmethod
    def detection_to_avstack(det_msg: RosDetection3D) -> BoxDetection:
        all_scores = [res.hypothesis.score for res in det_msg.results]
        all_types = [res.hypothesis.class_id for res in det_msg.results]
        max_score = np.max(all_scores)
        obj_type = all_types[np.argmax(all_scores)]
        bbox = GeometryBridge.box3d_to_avstack(det_msg.bbox, header=det_msg.header)
        det_out = BoxDetection(
            data=bbox,
            noise=None,
            source_identifier="0",
            reference=bbox.reference,
            score=max_score,
            obj_type=obj_type,
        )
        return det_out

    @staticmethod
    def detectionarray_to_avstack(dets_msg: RosDetection3DArray) -> DataContainer:
        timestamp = Bridge.rostime_to_time(dets_msg.header.stamp)
        dets = [
            DetectionBridge.detection_to_avstack(det) for det in dets_msg.detections
        ]
        return DataContainer(
            frame=0, timestamp=timestamp, data=dets, source_identifier="0"
        )

    ##########################################
    # AVstack --> ROS
    ##########################################

    @staticmethod
    def avstack_to_detection(det: BoxDetection, header=None) -> RosDetection3D:
        if header is None:
            header = Bridge.reference_to_header(det.box.reference)
        bbox = GeometryBridge.avstack_to_box3d(det.box, stamped=False)
        hypo = ObjectHypothesis(score=float(det.score), class_id=det.obj_type)
        hypo_pose = ObjectHypothesisWithPose(hypothesis=hypo)
        det_ros = RosDetection3D(header=header, results=[hypo_pose], bbox=bbox)
        return det_ros

    @staticmethod
    def avstack_to_detectionarray(
        dets: DataContainer, header=None
    ) -> RosDetection3DArray:
        dets_ros = [
            DetectionBridge.avstack_to_detection(det, header=header) for det in dets
        ]
        dets_ros_array = RosDetection3DArray(header=header, detections=dets_ros)
        return dets_ros_array
