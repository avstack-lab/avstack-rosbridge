from typing import List, Tuple, Union

import numpy as np
from avstack.geometry import (
    Acceleration,
    AngularVelocity,
    Attitude,
    Box3D,
    Position,
    Rotation,
    Vector,
    Velocity,
)
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray

from .base import Bridge


class GeometryBridge:

    avstack_to_ros_types = {
        Acceleration: "Vector3",
        AngularVelocity: "Vector3",
        Attitude: "Quaternion",
        Box3D: "BoundingBox3D",
        Position: "Point",
        Velocity: "Vector3",
    }

    ###################################################
    # ROS --> AVstack methods
    ###################################################

    @staticmethod
    def _to_avstack(
        aclass: Union[Vector, Rotation],
        data: np.ndarray,
        header: Union[Header, None],
    ) -> Union[Point, Quaternion, Vector3]:
        reference = Bridge.header_to_reference(header=header)
        return aclass(data, reference=reference)

    @classmethod
    def pose_to_avstack(
        cls,
        pose: Pose,
        header: Union[Header, None],
    ) -> Tuple[Position, Attitude]:
        position = cls.position_to_avstack(pose.position, header=header)
        attitude = cls.attitude_to_avstack(pose.orientation, header=header)
        return position, attitude

    @classmethod
    def position_to_avstack(
        cls,
        position: Point,
        header: Union[Header, None],
    ) -> Position:
        x = np.array([position.x, position.y, position.z])
        return cls._to_avstack(aclass=Position, data=x, header=header)

    @classmethod
    def attitude_to_avstack(
        cls,
        attitude: Quaternion,
        header: Union[Header, None],
    ) -> Attitude:
        q = np.quaternion(-attitude.w, attitude.x, attitude.y, attitude.z)
        return cls._to_avstack(aclass=Attitude, data=q, header=header)

    @classmethod
    def velocity_to_avstack(
        cls,
        velocity: Vector3,
        header: Union[Header, None],
    ) -> Velocity:
        vel = np.array([velocity.x, velocity.y, velocity.z])
        return cls._to_avstack(aclass=Velocity, data=vel, header=header)

    @classmethod
    def acceleration_to_avstack(
        cls,
        accel: Vector3,
        header: Union[Header, None],
    ) -> Acceleration:
        acc = np.array([accel.x, accel.y, accel.z])
        return cls._to_avstack(aclass=Acceleration, data=acc, header=header)

    @classmethod
    def angular_vel_to_avstack(
        cls,
        angular_velocity: Vector3,
        header: Union[Header, None],
    ) -> AngularVelocity:
        # avel = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])
        # TODO: fix this
        avel = np.quaternion(1)
        return cls._to_avstack(aclass=AngularVelocity, data=avel, header=header)

    @classmethod
    def box3d_to_avstack(
        cls,
        box: BoundingBox3D,
        header: Union[Header, None],
    ) -> Box3D:
        # reference = cls.header_to_reference(header=header, tf_buffer=tf_buffer)
        position, attitude = cls.pose_to_avstack(box.center, header=header)
        hwl = [box.size.z, box.size.y, box.size.x]  # TODO: is this the right order?
        return Box3D(position=position, attitude=attitude, hwl=hwl)

    @classmethod
    def box3d_array_to_avstack(
        cls,
        boxes: BoundingBox3DArray,
    ) -> List[Box3D]:
        header = boxes.header
        boxes = [
            box if isinstance(box, BoundingBox3D) else box.box for box in boxes.boxes
        ]
        boxes_av = [cls.box3d_to_avstack(box=box, header=header) for box in boxes]
        return boxes_av

    ###################################################
    # AVstack --> ROS methods
    ###################################################

    @classmethod
    def _to_ros_with_header(
        data: Union[Vector, Rotation],
    ) -> Union[Point, Vector3, Quaternion]:
        raise NotImplementedError("Need to implement with header")

    @classmethod
    def _to_ros_no_header(
        cls, data: Union[Vector, Rotation], out_type: str
    ) -> Union[Point, Vector3, Quaternion]:
        if out_type == "Point":
            return Point(x=data.x[0], y=data.x[1], z=data.x[2])
        elif out_type == "Vector3":
            if data:
                return Vector3(x=data.x[0], y=data.x[1], z=data.x[2])
            else:
                return Vector3()
        elif out_type == "Quaternion":
            if data:
                return Quaternion(x=data.qx, y=data.qy, z=data.qz, w=-data.qw)
            else:
                return Quaternion()
        else:
            raise NotImplementedError(cls.avstack_to_ros_types[type(data)])

    @classmethod
    def _to_ros(
        cls,
        data: Union[Vector, Rotation],
        stamped: bool,
        out_type: str,
    ) -> Union[Vector, Rotation]:
        if (
            (data is not None) and (data.reference is not None) and (stamped)
        ):  # cannot have header without a timestamp
            return cls._to_ros_with_header(data=data, out_type=out_type)
        else:
            return cls._to_ros_no_header(data=data, out_type=out_type)

    @classmethod
    def avstack_to_pose(cls, pos: Position, att: Attitude, stamped: bool) -> Pose:
        if stamped:
            raise NotImplementedError("Cannot do stamped pose yet")
        position = cls.avstack_to_position(pos, stamped=stamped)
        attitude = cls.avstack_to_attitude(att, stamped=stamped)
        return Pose(position=position, orientation=attitude)

    @classmethod
    def avstack_to_position(cls, position: Position, stamped: bool) -> Point:
        return cls._to_ros(data=position, stamped=stamped, out_type="Point")

    @classmethod
    def avstack_to_attitude(cls, attitude: Attitude, stamped: bool) -> Quaternion:
        return cls._to_ros(data=attitude, stamped=stamped, out_type="Quaternion")

    @classmethod
    def avstack_to_velocity(cls, velocity: Velocity, stamped: bool) -> Vector3:
        return cls._to_ros(data=velocity, stamped=stamped, out_type="Vector3")

    @classmethod
    def avstack_to_acceleration(cls, accel: Acceleration, stamped: bool) -> Vector3:
        return cls._to_ros(data=accel, stamped=stamped, out_type="Vector3")

    @classmethod
    def avstack_to_angular_vel(cls, avel: AngularVelocity, stamped: bool) -> Vector3:
        # TODO: fix this.....
        return Vector3()

    @classmethod
    def avstack_to_box3d(cls, box: Box3D, stamped: bool) -> BoundingBox3D:
        if stamped:
            raise NotImplementedError("Cannot do stamped box yet")
        if box:
            center = cls.avstack_to_pose(box.position, box.attitude, stamped=stamped)
            size = Vector3(
                x=float(box.l), y=float(box.w), z=float(box.h)
            )  # TODO: is this the right order?
            return BoundingBox3D(center=center, size=size)
        else:
            return BoundingBox3D()

    @classmethod
    def avstack_to_box3d_array(
        cls,
        boxes: List[Box3D],
        header=None,
    ) -> BoundingBox3DArray:
        if len(boxes) == 0:
            return BoundingBox3DArray()
        else:
            if not header:
                header = cls.reference_to_header(boxes[0].reference)
        boxes = [box if isinstance(box, Box3D) else box.box for box in boxes]  # HACK
        boxes_ros = [cls.avstack_to_box3d(box, stamped=False) for box in boxes]
        return BoundingBox3DArray(header=header, boxes=boxes_ros)
