from avstack.datastructs import DataContainer
from tf2_ros.buffer import Buffer
from vision_msgs.msg import BoundingBox3DArray

from .base import Bridge
from .geometry import GeometryBridge


class TrackBridge(Bridge):
    def __init__(self) -> None:
        self.geom_bridge = GeometryBridge()

    ##########################################
    # ROS --> AVstack
    ##########################################

    def tracks_to_avstack(
        self, trks_msg: BoundingBox3DArray, tf_buffer=None
    ) -> DataContainer:
        timestamp = self.rostime_to_time(trks_msg.header.stamp)
        dets = self.geom_bridge.box3d_array_to_avstack(trks_msg, tf_buffer=tf_buffer)
        return DataContainer(
            frame=-1, timestamp=timestamp, data=dets, source_identifier="0"
        )

    ##########################################
    # AVstack --> ROS
    ##########################################

    def avstack_to_tracks(
        self, dets: DataContainer, tf_buffer: Buffer | None = None
    ) -> BoundingBox3DArray:
        return self.geom_bridge.avstack_to_box3d_array(dets, tf_buffer=tf_buffer)
