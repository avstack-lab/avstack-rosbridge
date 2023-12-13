import numpy as np
from avstack.geometry import PassiveReferenceFrame, ReferenceFrame
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header


class Bridge:
    @staticmethod
    def ndarray_to_list(array: np.ndarray) -> list:
        return array.ravel().tolist()

    @staticmethod
    def list_to_2d_ndarray(list: list, shape=None):
        if not shape:
            l_sqrt = np.sqrt(len(list))
            if l_sqrt != int(l_sqrt):
                raise ValueError(
                    "If shape is None, then list length must be square-rootable but it's of length {}".format(
                        len(list)
                    )
                )
            else:
                shape = (int(l_sqrt), int(l_sqrt))
        return np.array(list).reshape(shape)

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

    @classmethod
    def header_to_reference(cls, header: Header) -> PassiveReferenceFrame:
        return PassiveReferenceFrame(
            frame_id=header.frame_id, timestamp=cls.rostime_to_time(header.stamp)
        )

    @classmethod
    def reference_to_header(cls, reference: PassiveReferenceFrame):
        return Header(
            frame_id=reference.frame_id, stamp=cls.time_to_rostime(reference.timestamp)
        )

    @classmethod
    def reference_to_tf2_stamped(cls, reference: ReferenceFrame) -> TransformStamped:
        header = Header(
            frame_id=reference.from_frame,
            stamp=cls.time_to_rostime(reference.timestamp),
        )
        transform = cls.reference_to_tf2(reference)
        return TransformStamped(
            header=header, child_frame_id=reference.to_frame, transform=transform
        )

    @staticmethod
    def reference_to_tf2(reference: ReferenceFrame) -> Transform:
        dx, dq = reference.transform
        translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
        rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
        return Transform(translation=translation, rotation=rotation)
