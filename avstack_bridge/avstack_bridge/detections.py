from avstack.datastructs import DataContainer
from tf2_ros.buffer import Buffer
from vision_msgs.msg import BoundingBox3DArray

from .base import Bridge
from .geometry import GeometryBridge


class DetectionBridge(Bridge):
    def __init__(self) -> None:
        self.geom_bridge = GeometryBridge()

    ##########################################
    # ROS --> AVstack
    ##########################################

    def detections_to_avstack(
        self, dets_msg: BoundingBox3DArray, tf_buffer=None
    ) -> DataContainer:
        timestamp = self.rostime_to_time(dets_msg.header.stamp)
        dets = self.geom_bridge.box3d_array_to_avstack(dets_msg, tf_buffer=tf_buffer)
        return DataContainer(
            frame=-1, timestamp=timestamp, data=dets, source_identifier="0"
        )

    ##########################################
    # AVstack --> ROS
    ##########################################

    def avstack_to_detections(
        self, dets: DataContainer, tf_buffer: Buffer | None = None, frame_override=None
    ) -> BoundingBox3DArray:
        return self.geom_bridge.avstack_to_box3d_array(
            dets, tf_buffer=tf_buffer, frame_override=frame_override
        )
