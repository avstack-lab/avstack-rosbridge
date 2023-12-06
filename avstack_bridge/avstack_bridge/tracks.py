from avstack.datastructs import DataContainer

from avstack_msgs.msg import ObjectStateArray

from .base import Bridge
from .objects import ObjectStateBridge


class TrackBridge:
    @staticmethod
    def tracks_to_avstack(trks_msg: ObjectStateArray) -> DataContainer:
        timestamp = Bridge.rostime_to_time(trks_msg.header.stamp)
        tracks = ObjectStateBridge.objectstatearray_to_avstack(trks_msg)
        return DataContainer(
            frame=0, timestamp=timestamp, data=tracks, source_identifier="0"
        )

    @staticmethod
    def avstack_to_tracks(tracks: DataContainer, header=None) -> ObjectStateArray:
        trks_msg = ObjectStateBridge.avstack_to_objecstatearray(
            obj_states=[trk.as_object() for trk in tracks],
            header=header,
        )
        return trks_msg
