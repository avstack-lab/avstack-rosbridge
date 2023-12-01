from avstack.datastructs import DataContainer
from tf2_ros.buffer import Buffer

from avstack_msgs.msg import ObjectStateArray

from .base import Bridge
from .objects import ObjectStateBridge


class TrackBridge(Bridge):
    def __init__(self) -> None:
        self.obj_bridge = ObjectStateBridge()

    ##########################################
    # ROS --> AVstack
    ##########################################

    def tracks_to_avstack(
        self, trks_msg: ObjectStateArray, tf_buffer=None
    ) -> DataContainer:
        timestamp = self.rostime_to_time(trks_msg.header.stamp)
        tracks = self.obj_bridge.objectstatearray_to_avstack(
            trks_msg, tf_buffer=tf_buffer
        )
        return DataContainer(
            frame=-1, timestamp=timestamp, data=tracks, source_identifier="0"
        )

    ##########################################
    # AVstack --> ROS
    ##########################################

    def avstack_to_tracks(
        self, tracks: DataContainer, tf_buffer: Buffer | None = None
    ) -> ObjectStateArray:
        trks_msg = self.obj_bridge.avstack_to_objecstatearray(
            obj_states=[trk.as_object() for trk in tracks],
            tf_buffer=tf_buffer,
        )
        return trks_msg
