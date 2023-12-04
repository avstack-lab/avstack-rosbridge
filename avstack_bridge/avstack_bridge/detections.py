from avstack.datastructs import DataContainer
from vision_msgs.msg import BoundingBox3DArray

from .base import Bridge
from .geometry import GeometryBridge


class DetectionBridge:
    @staticmethod
    def detections_to_avstack(dets_msg: BoundingBox3DArray) -> DataContainer:
        timestamp = Bridge.rostime_to_time(dets_msg.header.stamp)
        dets = GeometryBridge.box3d_array_to_avstack(dets_msg)
        return DataContainer(
            frame=0, timestamp=timestamp, data=dets, source_identifier="0"
        )

    @staticmethod
    def avstack_to_detections(dets: DataContainer, header=None) -> BoundingBox3DArray:
        return GeometryBridge.avstack_to_box3d_array(dets, header=header)
