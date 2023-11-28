import numpy as np
from avstack.geometry import (
    Acceleration,
    AngularVelocity,
    Attitude,
    Box3D,
    Position,
    ReferenceFrame,
    Rotation,
    Vector,
    Velocity,
)
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from vision_msgs.msg import BoundingBox3D

from .base import Bridge


class GeometryBridge(Bridge):

    ###################################################
    # ROS --> AVstack methods
    ###################################################

    @classmethod
    def _to_avstack(
        cls,
        aclass: Vector | Rotation,
        data: np.ndarray,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Point | Quaternion | Vector3:
        reference = cls.header_to_reference(header=header, tf_buffer=tf_buffer)
        return aclass(data, reference=reference)

    @classmethod
    def pose_to_avstack(
        cls,
        pose: Pose,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> (Position, Attitude):
        position = cls.position_to_avstack(
            pose.position, header=header, tf_buffer=tf_buffer
        )
        attitude = cls.attitude_to_avstack(
            pose.orientation, header=header, tf_buffer=tf_buffer
        )
        return position, attitude

    @classmethod
    def position_to_avstack(
        cls,
        position: Point,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Position:
        x = np.array([position.x, position.y, position.z])
        return cls._to_avstack(
            aclass=Position, data=x, header=header, tf_buffer=tf_buffer
        )

    @classmethod
    def attitude_to_avstack(
        cls,
        attitude: Quaternion,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Attitude:
        q = np.quaternion(attitude.x, attitude.y, attitude.z, attitude.w)
        return cls._to_avstack(
            aclass=Attitude, data=q, header=header, tf_buffer=tf_buffer
        )

    @classmethod
    def velocity_to_avstack(
        cls,
        velocity: Vector3,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Velocity:
        vel = np.array([velocity.x, velocity.y, velocity.z])
        return cls._to_avstack(
            aclass=Velocity, data=vel, header=header, tf_buffer=tf_buffer
        )

    @classmethod
    def acceleration_to_avstack(
        cls,
        accel: Vector3,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Acceleration:
        acc = np.array([accel.x, accel.y, accel.z])
        return cls._to_avstack(
            aclass=Acceleration, data=acc, header=header, tf_buffer=tf_buffer
        )

    @classmethod
    def angular_vel_to_avstack(
        cls,
        angular_velocity: Vector3,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> AngularVelocity:
        # avel = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])
        # TODO: fix this
        avel = np.quaternion(1)
        return cls._to_avstack(
            aclass=AngularVelocity, data=avel, header=header, tf_buffer=tf_buffer
        )

    @classmethod
    def box3d_to_avstack(
        cls,
        box: BoundingBox3D,
        header: Header | None,
        tf_buffer: Buffer | None,
    ) -> Box3D:
        reference = cls.header_to_reference(header=header, tf_buffer=tf_buffer)
        position, attitude = cls.pose_to_avstack(
            box.center, header=header, tf_buffer=tf_buffer
        )
        hwl = [box.size.z, box.size.y, box.size.x]  # TODO: is this the right order?
        return Box3D(position=position, attitude=attitude, hwl=hwl)

    ###################################################
    # AVstack --> ROS methods
    ###################################################

    @classmethod
    def _to_ros_with_header(
        cls,
        aclass: Point | Quaternion | Vector3,
        data: np.ndarray,
        reference: ReferenceFrame,
        **kwargs,
    ) -> Vector | Rotation:
        raise NotImplementedError("Need to implement with header")

    @staticmethod
    def _to_ros_no_header(
        aclass: Point | Quaternion | Vector3, data: np.ndarray, **kwargs
    ) -> Vector | Rotation:
        if aclass == "point":
            return Point(x=data[0], y=data[1], z=data[2])
        elif aclass == "vector3":
            return Vector3(x=data[0], y=data[1], z=data[2])
        elif aclass == "quaternion":
            return Quaternion(x=data.x, y=data.y, z=data.z, w=data.w)
        else:
            raise NotImplementedError(aclass)

    @classmethod
    def _to_ros(
        cls,
        aclass: Point | Quaternion | Vector3,
        data: np.ndarray,
        reference: ReferenceFrame | None,
        stamped: bool,
        **kwargs,
    ) -> Vector | Rotation:
        if (reference is not None) and (
            stamped
        ):  # cannot have header without a timestamp
            return cls._to_ros_with_header(
                aclass=aclass, data=data, reference=reference
            )
        else:
            return cls._to_ros_no_header(aclass=aclass, data=data)

    @classmethod
    def avstack_to_pose(cls, pos: Position, att: Attitude, stamped: bool) -> Pose:
        if stamped:
            raise NotImplementedError("Cannot do stamped pose yet")
        position = cls.avstack_to_position(pos, stamped=stamped)
        attitude = cls.avstack_to_attitude(att, stamped=stamped)
        return Pose(position=position, orientation=attitude)

    @classmethod
    def avstack_to_position(cls, position: Position, stamped: bool) -> Point:
        return cls._to_ros(
            aclass="point",
            data=position.x,
            reference=position.reference,
            stamped=stamped,
        )

    @classmethod
    def avstack_to_attitude(cls, attitude: Attitude, stamped: bool) -> Quaternion:
        return cls._to_ros(
            aclass="quaternion",
            data=attitude.q,
            reference=attitude.reference,
            stamped=stamped,
        )

    @classmethod
    def avstack_to_velocity(cls, velocity: Velocity, stamped: bool) -> Vector3:
        return cls._to_ros(
            aclass="vector3",
            data=velocity.x,
            reference=velocity.reference,
            stamped=stamped,
        )

    @classmethod
    def avstack_to_acceleration(cls, accel: Acceleration, stamped: bool) -> Vector3:
        return cls._to_ros(
            aclass="vector3", data=accel.x, reference=accel.reference, stamped=stamped
        )

    @classmethod
    def avstack_to_angular_vel(cls, avel: AngularVelocity, stamped: bool) -> Vector3:
        # TODO: fix this.....
        return Vector3()
        # return cls._to_ros(
        #     aclass="vector3", data=avel.x, reference=avel.reference, stamped=stamped
        # )

    @classmethod
    def avstack_to_box3d(cls, box: Box3D, stamped: bool) -> BoundingBox3D:
        if stamped:
            raise NotImplementedError("Cannot do stamped box yet")
        center = cls.avstack_to_pose(box.position, box.attitude, stamped=stamped)
        size = Vector3(
            x=float(box.l), y=float(box.w), z=float(box.h)
        )  # TODO: is this the right order?
        return BoundingBox3D(center=center, size=size)
