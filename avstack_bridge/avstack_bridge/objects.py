from avstack.environment.objects import ObjectState as ObjectStateAV
from avstack.geometry import GlobalOrigin3D
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

from avstack_msgs.msg import ObjectState, ObjectStateArray, ObjectStateStamped

from .base import Bridge
from .geometry import GeometryBridge


class ObjectStateBridge(Bridge):
    def __init__(self) -> None:
        self.geom_bridge = GeometryBridge()

    def header_from_avstack(self, obj_state: ObjectStateAV, tf_buffer=None) -> Header:
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
        return header

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

        obj_state = ObjectStateAV(obj_type=obj.obj_type)
        obj_state.set(
            t=timestamp,
            position=self.geom_bridge.position_to_avstack(
                obj.position, header=header, tf_buffer=tf_buffer
            ),
            attitude=self.geom_bridge.attitude_to_avstack(
                obj.attitude, header=header, tf_buffer=tf_buffer
            ),
            velocity=self.geom_bridge.velocity_to_avstack(
                obj.linear_velocity, header=header, tf_buffer=tf_buffer
            ),
            acceleration=self.geom_bridge.acceleration_to_avstack(
                obj.linear_acceleration, header=header, tf_buffer=tf_buffer
            ),
            angular_velocity=self.geom_bridge.angular_vel_to_avstack(
                obj.angular_velocity, header=header, tf_buffer=tf_buffer
            ),
            box=self.geom_bridge.box3d_to_avstack(
                obj.box, header=header, tf_buffer=tf_buffer
            ),
        )
        return obj_state

    def avstack_to_objectstate(
        self, obj_state: ObjectStateAV, tf_buffer=None
    ) -> ObjectStateStamped:
        header = self.header_from_avstack(obj_state, tf_buffer)
        state = ObjectState(
            obj_type=obj_state.obj_type,
            position=self.geom_bridge.avstack_to_position(
                obj_state.position, stamped=False
            ),
            attitude=self.geom_bridge.avstack_to_attitude(
                obj_state.attitude, stamped=False
            ),
            linear_velocity=self.geom_bridge.avstack_to_velocity(
                obj_state.velocity, stamped=False
            ),
            angular_velocity=self.geom_bridge.avstack_to_angular_vel(
                obj_state.angular_velocity, stamped=False
            ),
            linear_acceleration=self.geom_bridge.avstack_to_acceleration(
                obj_state.acceleration, stamped=False
            ),
            angular_acceleration=Vector3(),  # TODO: we don't have this in avstack
            box=self.geom_bridge.avstack_to_box3d(obj_state.box, stamped=False),
        )
        obj_state = ObjectStateStamped(header=header, state=state)
        return obj_state

    def avstack_to_objecstatearray(
        self, obj_states, tf_buffer=None
    ) -> ObjectStateArray:
        if len(obj_states) == 0:
            raise NotImplementedError("How do we get header without objects?")
        header = self.header_from_avstack(obj_states[0], tf_buffer)
        states = [
            self.avstack_to_objectstate(obj, tf_buffer).state for obj in obj_states
        ]
        return ObjectStateArray(header=header, states=states)
