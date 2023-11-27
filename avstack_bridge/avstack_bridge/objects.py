import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Vector3
from mar_msgs.msg import ObjectState, ObjectStateStamped, ObjectStateArray

from .base import Bridge
from .geometry import GeometryBridge
from avstack.environment.objects import ObjectState as ObjectStateAV
from avstack.geometry import GlobalOrigin3D


class ObjectStateBridge(Bridge):
    def __init__(self) -> None:
        self.geom_bridge = GeometryBridge()

    def objectstate_to_avstack(self, msg_obj, tf_buffer=None) -> ObjectStateAV:
        if isinstance(msg_obj, ObjectState):
            obj = msg_obj
            timestamp = None
            header = Header(frame_id="world")
        elif isinstance(msg_obj, ObjectStateStamped):
            obj = msg_obj.state
            timestamp = self.rostime_to_time(msg_obj.header.stamp)
            header = msg_obj.header
        else:
            raise NotImplementedError(type(msg_obj))


        obj_state = ObjectStateAV(obj.obj_type)
        obj_state.set(
            t=timestamp,
            position=self.geom_bridge.position_to_avstack(obj.position, header=header),
            attitude=self.geom_bridge.attitude_to_avstack(obj.atttiude, header=header),
            velocity=self.geom_bridge.velocity_to_avstack(obj.linear_velocity, header=header),
            acceleration=self.geom_bridge.acceleration_to_avstack(obj.linear_acceleration, header=header),
            angular_velocity=self.geom_bridge.angular_vel_to_avstack(obj.angular_velocity, header=header),
        )

    def avstack_to_objectstate(self, obj_state: ObjectStateAV, tf_buffer=None) -> ObjectStateStamped:
        if obj_state.reference != GlobalOrigin3D:
            if tf_buffer is None:
                raise RuntimeError("tf buffer must exist if origin is not world")
            else:
                frame_id = None  # TODO
                raise
        else:
            frame_id = "world"
            rostime = self.time_to_rostime(obj_state.reference.timestamp)
            header = Header(stamp=rostime, frame_id=frame_id)
        
        position = Point(x=obj_state.position.x[0], y=obj_state.position.x[1], z=obj_state.position.x[2])
        orientation = Quaternion(x=obj_state.qx, y=obj_state.qy, z=obj_state.qz, w=obj_state.qw)
        pose = Pose(position=position, orientation=orientation)
        linear = Vector3(x=obj_state.velocity.x[0], y=obj_state.velocity.x[1], z=obj_state.velocity.x[2])
        if not np.all(obj_state.angular_velocity.x == 0):
            raise NotImplementedError("Need to settle on angular velocity conventions")
        angular = Vector3(x=0, y=0, z=0)  # TODO
        twist = Twist(linear=linear, angular=angular)
        return PoseStamped(header=header, pose=pose), TwistStamped(header=header, twist=twist)

    def avstack_to_objecstatearray(self, obj_states, tf_buffer=None) -> "tuple[PoseArray, TwistArray]"
