import numpy as np
from avstack.geometry import GlobalOrigin3D, ReferenceFrame
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class Bridge:
    # def __init__(self) -> None:
    #     self.tf_buffer = Buffer()
    #     self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def time_to_rostime(timestamp: float) -> Time:
        ts = Time(
            sec=int(timestamp),
            nanosec=int(1e9 * (timestamp - int(timestamp))),
        )
        return ts

    @staticmethod
    def rostime_to_time(msg: Time) -> float:
        ts = float(msg.sec + msg.nanosec / 1e9)
        return ts

    def frameid_to_reference(
        self, to_frame: str, from_frame: str, time: Time
    ) -> ReferenceFrame:
        tr = self.frameid_to_tf2(to_frame=to_frame, from_frame=from_frame, time=time)
        if from_frame != "world":
            tran_world_2_from = self.frameid_to_tf2(
                to_frame=from_frame, from_frame="world", time=time
            )
        else:
            tran_world_2_from = GlobalOrigin3D
        ref = ReferenceFrame(
            x=np.array(
                [
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z,
                ]
            ),
            q=np.quaternion(
                [
                    tr.transform.rotation.x,
                    tr.transform.rotation.y,
                    tr.transform.rotation.z,
                    tr.transform.rotation.w,
                ]
            ),
            reference=tran_world_2_from,
            from_frame=from_frame,
            to_frame=to_frame,
            timestamp=self.rostime_to_time(time),
        )
        return ref

    def frameid_to_tf2(
        self, tf_buffer: Buffer, to_frame: str, from_frame: str, time: Time
    ) -> TransformStamped:
        try:
            tran = tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                time,
            )
        except TransformException as e:
            raise e
        else:
            return tran

    def reference_to_tf2(self, reference: ReferenceFrame) -> Transform:
        translation = Vector3(x=reference.x[0], y=reference.x[1], z=reference.z[2])
        rotation = Quaternion(
            x=reference.q[0], y=reference.q[1], z=reference.q[2], w=reference.q[3]
        )
        return Transform(translation=translation, rotation=rotation)

    def reference_to_header(
        self, reference: ReferenceFrame, timestamp: float
    ) -> Header:
        stamp = self.time_to_rostime(timestamp)
        frame_id = self.reference_to_frameid(reference)
        return Header(stamp=stamp, frame_id=frame_id)

    def reference_to_frameid(self, reference: ReferenceFrame) -> str:
        return reference.to_frame

    def header_to_reference(self, header: Header) -> ReferenceFrame:
        return self.frameid_to_reference(header.frame_id)
