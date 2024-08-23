from typing import List, Tuple, Union

import numpy as np
from avstack.geometry import (
    Acceleration,
    AngularVelocity,
    Attitude,
    Box3D,
    Polygon,
    Position,
    Rotation,
    Vector,
    Velocity,
)
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import Point32 as RosPoint32
from geometry_msgs.msg import PointStamped as RosPointStamped
from geometry_msgs.msg import Polygon as RosPolygon
from geometry_msgs.msg import PolygonStamped as RosPolygonStamped
from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import QuaternionStamped as RosQuaternionStamped
from geometry_msgs.msg import Vector3 as RosVector3
from geometry_msgs.msg import Vector3Stamped as RosVector3Stamped
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D as RosBoundingBox3D
from vision_msgs.msg import BoundingBox3DArray as RosBoundingBox3DArray

from .base import Bridge


class GeometryBridge:

    avstack_to_ros_types = {
        Acceleration: "RosVector3",
        AngularVelocity: "RosVector3",
        Attitude: "RosQuaternion",
        Box3D: "RosBoundingBox3D",
        Position: "RosPoint",
        Velocity: "RosVector3",
    }

    ###################################################
    # ROS --> AVstack methods
    ###################################################

    @staticmethod
    def _to_avstack(
        aclass: Union[Vector, Rotation],
        data: np.ndarray,
        header: Union[Header, None],
    ) -> Union[RosPoint, RosQuaternion, RosVector3]:
        reference = Bridge.header_to_reference(header=header)
        return aclass(data, reference=reference)

    @classmethod
    def pose_to_avstack(
        cls,
        pose: RosPose,
        header: Union[Header, None],
    ) -> Tuple[Position, Attitude]:
        position = cls.position_to_avstack(pose.position, header=header)
        attitude = cls.attitude_to_avstack(pose.orientation, header=header)
        return position, attitude

    @classmethod
    def position_to_avstack(
        cls,
        position: RosPoint,
        header: Union[Header, None],
    ) -> Position:
        x = np.array([position.x, position.y, position.z])
        return cls._to_avstack(aclass=Position, data=x, header=header)

    @classmethod
    def attitude_to_avstack(
        cls,
        attitude: RosQuaternion,
        header: Union[Header, None],
    ) -> Attitude:
        q = np.quaternion(-attitude.w, attitude.x, attitude.y, attitude.z)
        return cls._to_avstack(aclass=Attitude, data=q, header=header)

    @classmethod
    def velocity_to_avstack(
        cls,
        velocity: RosVector3,
        header: Union[Header, None],
    ) -> Velocity:
        vel = np.array([velocity.x, velocity.y, velocity.z])
        return cls._to_avstack(aclass=Velocity, data=vel, header=header)

    @classmethod
    def acceleration_to_avstack(
        cls,
        accel: RosVector3,
        header: Union[Header, None],
    ) -> Acceleration:
        acc = np.array([accel.x, accel.y, accel.z])
        return cls._to_avstack(aclass=Acceleration, data=acc, header=header)

    @classmethod
    def angular_vel_to_avstack(
        cls,
        angular_velocity: RosVector3,
        header: Union[Header, None],
    ) -> AngularVelocity:
        # avel = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])
        # TODO: fix this
        avel = np.quaternion(1)
        return cls._to_avstack(aclass=AngularVelocity, data=avel, header=header)

    @classmethod
    def box3d_to_avstack(
        cls,
        box: RosBoundingBox3D,
        header: Union[Header, None],
    ) -> Box3D:
        position, attitude = cls.pose_to_avstack(box.center, header=header)
        hwl = [box.size.z, box.size.y, box.size.x]  # TODO: is this the right order?
        return Box3D(position=position, attitude=attitude, hwl=hwl, where_is_t="center")

    @classmethod
    def box3d_array_to_avstack(
        cls,
        boxes: RosBoundingBox3DArray,
    ) -> List[Box3D]:
        header = boxes.header
        boxes = [
            box if isinstance(box, RosBoundingBox3D) else box.box for box in boxes.boxes
        ]
        boxes_av = [cls.box3d_to_avstack(box=box, header=header) for box in boxes]
        return boxes_av

    @classmethod
    def polygon_to_avstack(
        cls,
        polygon: Union[RosPolygon, RosPolygonStamped],
        header: Union[Header, None] = None,
    ) -> Polygon:
        reference = Bridge.header_to_reference(
            header if header is not None else polygon.header
        )
        if isinstance(polygon, RosPolygonStamped):
            boundary = np.array([[pt.x, pt.y] for pt in polygon.polygon.points])
        else:
            boundary = np.array([[pt.x, pt.y] for pt in polygon.points])
        polygon_av = Polygon(boundary=boundary, reference=reference)
        return polygon_av

    ###################################################
    # AVstack --> ROS methods
    ###################################################

    @classmethod
    def _to_ros_with_header(
        cls, data: Union[Vector, Rotation], out_type: str
    ) -> Union[RosPoint, RosVector3, RosQuaternion]:
        header = Bridge.reference_to_header(data.reference)
        val = cls._to_ros_no_header(data, out_type)
        if out_type == "Point":
            return RosPointStamped(header=header, point=val)
        elif out_type == "Vector3":
            return RosVector3Stamped(header=header, vector=val)
        elif out_type == "Quaternion":
            return RosQuaternionStamped(header=header, quaternion=val)
        else:
            raise NotImplementedError(out_type)

    @classmethod
    def _to_ros_no_header(
        cls, data: Union[Vector, Rotation], out_type: str
    ) -> Union[RosPoint, RosVector3, RosQuaternion]:
        if out_type == "Point":
            return RosPoint(x=data.x[0], y=data.x[1], z=data.x[2])
        elif out_type == "Vector3":
            if data:
                return RosVector3(x=data.x[0], y=data.x[1], z=data.x[2])
            else:
                return RosVector3()
        elif out_type == "Quaternion":
            if data:
                return RosQuaternion(x=data.qx, y=data.qy, z=data.qz, w=-data.qw)
            else:
                return RosQuaternion()
        else:
            raise NotImplementedError(out_type)

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
    def avstack_to_pose(
        cls, pos: Position, att: Attitude, stamped: bool
    ) -> Union[RosPose, RosPoseStamped]:
        """Pose is unique in that ros considers it synonymous with a frame/transform"""
        pose = RosPose(
            position=cls.avstack_to_position(pos, stamped=False),
            orientation=cls.avstack_to_attitude(att, stamped=False),
        )
        if stamped:
            header = Bridge.reference_to_header(pos.reference)
            pose = RosPoseStamped(header=header, pose=pose)
        return pose

    @classmethod
    def avstack_to_position(cls, position: Position, stamped: bool) -> RosPoint:
        return cls._to_ros(data=position, stamped=stamped, out_type="Point")

    @classmethod
    def avstack_to_attitude(cls, attitude: Attitude, stamped: bool) -> RosQuaternion:
        return cls._to_ros(data=attitude, stamped=stamped, out_type="Quaternion")

    @classmethod
    def avstack_to_velocity(cls, velocity: Velocity, stamped: bool) -> RosVector3:
        return cls._to_ros(data=velocity, stamped=stamped, out_type="Vector3")

    @classmethod
    def avstack_to_acceleration(cls, accel: Acceleration, stamped: bool) -> RosVector3:
        return cls._to_ros(data=accel, stamped=stamped, out_type="Vector3")

    @classmethod
    def avstack_to_angular_vel(cls, avel: AngularVelocity, stamped: bool) -> RosVector3:
        # TODO: fix this.....
        return RosVector3()

    @classmethod
    def avstack_to_box3d(cls, box: Box3D, stamped: bool) -> RosBoundingBox3D:
        if stamped:
            raise NotImplementedError("Cannot do stamped box yet")
        if box:
            center = cls.avstack_to_pose(box.center, box.attitude, stamped=stamped)
            size = RosVector3(
                x=float(box.l), y=float(box.w), z=float(box.h)
            )  # TODO: is this the right order?
            return RosBoundingBox3D(center=center, size=size)
        else:
            return RosBoundingBox3D()

    @classmethod
    def avstack_to_box3d_array(
        cls,
        boxes: List[Box3D],
        header=None,
    ) -> RosBoundingBox3DArray:
        if len(boxes) == 0:
            return RosBoundingBox3DArray()
        else:
            if not header:
                header = Bridge.reference_to_header(boxes[0].reference)
        boxes = [box if isinstance(box, Box3D) else box.box for box in boxes]  # HACK
        boxes_ros = [cls.avstack_to_box3d(box, stamped=False) for box in boxes]
        return RosBoundingBox3DArray(header=header, boxes=boxes_ros)

    @classmethod
    def avstack_to_polygon(
        cls,
        polygon: Polygon,
        stamped: bool,
        header=None,
    ) -> RosPolygonStamped:
        polygon_ros = RosPolygon(
            points=[RosPoint32(x=pt[0], y=pt[1]) for pt in polygon.boundary]
        )
        if stamped:
            if not header:
                header = Bridge.reference_to_header(polygon.reference)
            polygon_ros = RosPolygonStamped(header=header, polygon=polygon_ros)
        return polygon_ros
