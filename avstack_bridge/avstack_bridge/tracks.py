from typing import Union

from avstack.datastructs import DataContainer
from avstack.modules.tracking.tracks import BasicBoxTrack3D
from std_msgs.msg import Header

from avstack_msgs.msg import BoxTrack, BoxTrackArray, BoxTrackStamped, ObjectStateArray

from .base import Bridge
from .geometry import GeometryBridge
from .objects import ObjectStateBridge


class TrackBridge:
    @classmethod
    def tracks_to_avstack(
        cls, trks_msg: Union[BoxTrackArray, ObjectStateArray]
    ) -> DataContainer:
        timestamp = Bridge.rostime_to_time(trks_msg.header.stamp)
        try:
            tracks = [
                cls.boxtrack_to_avstack(trk_msg=trk, header=trks_msg.header)
                for trk in trks_msg.tracks
            ]
        except AttributeError:
            tracks = ObjectStateBridge.objectstatearray_to_avstack(trks_msg)

        return DataContainer(
            frame=0, timestamp=timestamp, data=tracks, source_identifier="0"
        )

    @staticmethod
    def boxtrack_to_avstack(
        trk_msg: Union[BoxTrack, BoxTrackStamped], header: Union[Header, None] = None
    ) -> BasicBoxTrack3D:
        if isinstance(trk_msg, BoxTrackStamped):
            if header is not None:
                raise RuntimeError("Passed stamped and a header...")
            header = trk_msg.header
            trk_msg = trk_msg.track
        return BasicBoxTrack3D(
            t0=0,
            box3d=GeometryBridge.box3d_to_avstack(trk_msg.box, header=header),
            reference=Bridge.header_to_reference(header),
            obj_type=trk_msg.obj_type if trk_msg.obj_type else None,
            ID_force=None if trk_msg.identifier == 0 else trk_msg.identifier,
            v=GeometryBridge.velocity_to_avstack(
                velocity=trk_msg.velocity, header=header
            ),
            P=Bridge.list_to_2d_ndarray(trk_msg.p),
            t=Bridge.rostime_to_time(header.stamp),
            coast=trk_msg.coast,
            n_updates=trk_msg.n_updates,
            age=trk_msg.age,
        )

    @staticmethod
    def avstack_to_boxtrack(track: BasicBoxTrack3D) -> BoxTrack:
        box = GeometryBridge.avstack_to_box3d(box=track.box3d, stamped=False)
        vel = GeometryBridge.avstack_to_velocity(velocity=track.velocity, stamped=False)
        return BoxTrack(
            obj_type=track.obj_type if track.obj_type else "",
            box=box,
            velocity=vel,
            p=Bridge.ndarray_to_list(track.P),
            n_updates=track.n_updates,
            age=track.age,
            coast=float(track.coast),
            identifier=track.ID,
        )

    @classmethod
    def avstack_to_boxtrack_stamped(
        cls, track: BasicBoxTrack3D, header=None
    ) -> BoxTrackStamped:
        if not header:
            header = Bridge.reference_to_header(track.reference)
        return BoxTrackStamped(header=header, track=cls.avstack_to_boxtrack(track))

    @classmethod
    def avstack_to_tracks(
        cls, tracks: DataContainer, header=None, default_type=ObjectStateArray
    ) -> Union[ObjectStateArray, BoxTrackArray]:
        if len(tracks) > 0:
            if not header:
                header = Bridge.reference_to_header(tracks.reference)
            try:
                trks_msg = BoxTrackArray(
                    header=header,
                    tracks=[cls.avstack_to_boxtrack(trk) for trk in tracks],
                )
            except AttributeError:
                trks_msg = ObjectStateBridge.avstack_to_objecstatearray(
                    obj_states=[trk.as_object() for trk in tracks],
                    header=header,
                )
        else:
            trks_msg = default_type(header=header)

        return trks_msg
