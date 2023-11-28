import numpy as np
from avstack.geometry import GlobalOrigin3D, ReferenceFrame
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class Bridge:

    ############################################
    # TIME
    ############################################

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

    ############################################
    # REFERENCE -- to AVstack
    ############################################

    @classmethod
    def header_to_reference(
        cls,
        header: Header | None,
        tf_buffer: Buffer,
        from_frame: str = "world",
    ) -> ReferenceFrame:
        if (header is None) or (header.frame_id == "world"):
            reference = GlobalOrigin3D
            if header is not None:
                reference.timestamp = cls.rostime_to_time(header.stamp)
        else:
            reference = cls.frameid_to_reference(
                to_frame=header.frame_id,
                from_frame=from_frame,
                time=header.stamp,
                tf_buffer=tf_buffer,
            )
        return reference

    @classmethod
    def frameid_to_reference(
        cls,
        to_frame: str,
        from_frame: str,
        time: Time,
        tf_buffer: Buffer,
    ) -> ReferenceFrame:
        tr = cls.frameid_to_tf2(to_frame=to_frame, from_frame=from_frame, time=time)
        if from_frame != "world":
            tran_world_2_from = cls.frameid_to_tf2(
                to_frame=from_frame,
                from_frame="world",
                time=time,
                tf_buffer=tf_buffer,
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
            timestamp=cls.rostime_to_time(time),
        )
        return ref

    ############################################
    # REFERENCE -- to ROS
    ############################################

    @classmethod
    def frameid_to_tf2(
        cls,
        to_frame: str,
        from_frame: str,
        time: Time,
        tf_buffer: Buffer,
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

    @classmethod
    def reference_to_tf2(cls, reference: ReferenceFrame) -> Transform:
        translation = Vector3(x=reference.x[0], y=reference.x[1], z=reference.x[2])
        rotation = Quaternion(
            x=reference.q.x, y=reference.q.y, z=reference.q.z, w=reference.q.w
        )
        return Transform(translation=translation, rotation=rotation)

    @classmethod
    def reference_to_tf2_stamped(cls, reference: ReferenceFrame) -> TransformStamped:
        header = cls.reference_to_header(reference)
        transform = cls.reference_to_tf2(reference)
        return TransformStamped(
            header=header, child_frame_id=reference.to_frame, transform=transform
        )

    @classmethod
    def reference_to_header(
        cls, reference: ReferenceFrame, timestamp: float | None = None
    ) -> Header:
        stamp = cls.time_to_rostime(
            reference.timestamp if timestamp is None else timestamp
        )
        frame_id = cls.reference_to_frameid(reference)
        return Header(stamp=stamp, frame_id=frame_id)

    @classmethod
    def reference_to_frameid(cls, reference: ReferenceFrame) -> str:
        if reference.is_global_origin:
            raise RuntimeError("Not supposed to pass the global origin to this")
        else:
            return reference.from_frame
