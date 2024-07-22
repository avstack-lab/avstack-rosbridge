from avstack.datastructs import DataContainer
from avstack.modules.perception.detections import BoxDetection
from vision_msgs.msg import Detection3D as RosDetection3D
from vision_msgs.msg import Detection3DArray as RosDetection3DArray

from .base import Bridge
from .geometry import GeometryBridge


class DetectionBridge:

    ##########################################
    # ROS --> AVstack
    ##########################################

    @staticmethod
    def detection_to_avstack(det_msg: RosDetection3D) -> BoxDetection:
        timestamp = Bridge.rostime_to_time(det_msg.header.stamp)
        bbox = GeometryBridge.box3d_to_avstack(det_msg.bbox)
        det_out = BoxDetection(
            frame=0, timestamp=timestamp, box=bbox, source_identifier="0"
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
    def avstack_to_detection(det: BoxDetection) -> RosDetection3D:
        header = Bridge.reference_to_header(det.box.reference)
        results = []  # TODO
        bbox = GeometryBridge.avstack_to_box3d(det.box)
        RosDetection3D(header=header, results=results, bbox=bbox)

    @staticmethod
    def avstack_to_detectionarray(
        dets: DataContainer, header=None
    ) -> RosDetection3DArray:
        dets_ros = [DetectionBridge.avstack_to_detection(det) for det in dets]
        dets_ros_array = RosDetection3DArray(header=header, detections=dets_ros)
        return dets_ros_array
