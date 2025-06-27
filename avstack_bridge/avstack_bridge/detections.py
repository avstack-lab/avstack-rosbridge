from typing import Union

import numpy as np
from avstack.datastructs import DataContainer
from avstack.geometry import Box2D
from avstack.modules.perception.detections import BoxDetection
from vision_msgs.msg import Detection2D as RosDetection2D
from vision_msgs.msg import Detection2DArray as RosDetection2DArray
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
    def detection_to_avstack(
        det_msg: Union[RosDetection2D, RosDetection3D],
        noise: np.ndarray,
    ) -> BoxDetection:

        # get common attributes
        all_scores = [res.hypothesis.score for res in det_msg.results]
        all_types = [res.hypothesis.class_id for res in det_msg.results]
        max_score = np.max(all_scores)
        obj_type = all_types[np.argmax(all_scores)]

        # convert box to box2d or box3d
        if isinstance(det_msg, RosDetection2D):
            bbox = GeometryBridge.box2d_to_avstack(det_msg.bbox, header=det_msg.header)
        else:
            bbox = GeometryBridge.box3d_to_avstack(det_msg.bbox, header=det_msg.header)

        # package up box detection
        det_out = BoxDetection(
            data=bbox,
            noise=noise,
            source_identifier="0",
            reference=bbox.reference,
            score=max_score,
            obj_type=obj_type,
        )
        return det_out

    @staticmethod
    def detectionarray_to_avstack(
        dets_msg: Union[RosDetection2DArray, RosDetection3DArray],
        noise: np.ndarray,
    ) -> DataContainer:
        timestamp = Bridge.rostime_to_time(dets_msg.header.stamp)
        dets = [
            DetectionBridge.detection_to_avstack(det, noise=noise)
            for det in dets_msg.detections
        ]
        return DataContainer(
            frame=0, timestamp=timestamp, data=dets, source_identifier="0"
        )

    ##########################################
    # AVstack --> ROS
    ##########################################

    @staticmethod
    def avstack_to_detection(
        det: BoxDetection, header=None
    ) -> Union[RosDetection2D, RosDetection3D]:
        if header is None:
            header = Bridge.reference_to_header(det.box.reference)

        # score and class ids
        hypo = ObjectHypothesis(score=float(det.score), class_id=det.obj_type)
        hypo_pose = ObjectHypothesisWithPose(hypothesis=hypo)

        # convert box to box2d or box3d
        if isinstance(det.box, Box2D):
            bbox = GeometryBridge.avstack_to_box2d(det.box, stamped=False)
            det_ros = RosDetection2D(header=header, results=[hypo_pose], bbox=bbox)
        else:
            bbox = GeometryBridge.avstack_to_box3d(det.box, stamped=False)
            det_ros = RosDetection3D(header=header, results=[hypo_pose], bbox=bbox)

        return det_ros

    @staticmethod
    def avstack_to_detectionarray(
        dets: DataContainer, det_is_2d: bool, header=None
    ) -> Union[RosDetection2DArray, RosDetection3DArray]:
        dets_ros = [
            DetectionBridge.avstack_to_detection(det, header=header) for det in dets
        ]
        if det_is_2d:
            dets_ros_array = RosDetection2DArray(header=header, detections=dets_ros)
        else:
            dets_ros_array = RosDetection3DArray(header=header, detections=dets_ros)
        return dets_ros_array
