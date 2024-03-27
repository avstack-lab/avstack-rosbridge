from typing import List, Union

from avstack.datastructs import DataContainer
from avstack.environment.objects import ObjectState as ObjectStateAV
from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg import Header

from avstack_msgs.msg import ObjectState, ObjectStateArray, ObjectStateStamped

from .base import Bridge
from .geometry import GeometryBridge


class ObjectStateBridge:

    ##########################################
    # ROS --> AVstack
    ##########################################

    @staticmethod
    def objectstate_to_avstack(
        msg_obj: Union[ObjectState, ObjectStateStamped]
    ) -> ObjectStateAV:
        if isinstance(msg_obj, ObjectState):
            obj = msg_obj
            timestamp = None
            header = Header(frame_id="world")
        elif isinstance(msg_obj, ObjectStateStamped):
            obj = msg_obj.state
            timestamp = Bridge.rostime_to_time(msg_obj.header.stamp)
            header = msg_obj.header
        else:
            raise NotImplementedError(type(msg_obj))

        position, attitude = GeometryBridge.pose_to_avstack(obj.pose, header=header)

        obj_state = ObjectStateAV(obj_type=obj.obj_type)
        obj_state.set(
            t=timestamp,
            position=position,
            attitude=attitude,
            velocity=GeometryBridge.velocity_to_avstack(
                obj.twist.linear,
                header=header,
            ),
            acceleration=GeometryBridge.acceleration_to_avstack(
                obj.linear_acceleration,
                header=header,
            ),
            angular_velocity=GeometryBridge.angular_vel_to_avstack(
                obj.twist.angular,
                header=header,
            ),
            box=GeometryBridge.box3d_to_avstack(
                obj.box,
                header=header,
            ),
        )
        return obj_state

    @classmethod
    def objectstatearray_to_avstack(
        cls,
        msg_objarray: ObjectStateArray,
    ) -> List[ObjectStateAV]:
        header = msg_objarray.header
        obj_states = []
        for obj in msg_objarray.states:
            msg_obj = ObjectStateStamped(header=header, state=obj)
            obj_state = cls.objectstate_to_avstack(
                msg_obj=msg_obj,
            )
            obj_states.append(obj_state)
        timestamp = Bridge.rostime_to_time(header.stamp)
        obj_states = DataContainer(
            frame=0,
            timestamp=timestamp,
            data=obj_states,
            source_identifier=header.frame_id,
        )
        return obj_states

    ##########################################
    # AVstack --> ROS
    ##########################################

    @staticmethod
    def avstack_to_objectstate(obj_state: ObjectStateAV) -> ObjectState:
        position = GeometryBridge.avstack_to_position(obj_state.position, stamped=False)
        orientation = GeometryBridge.avstack_to_attitude(
            obj_state.attitude, stamped=False
        )
        linear = GeometryBridge.avstack_to_velocity(obj_state.velocity, stamped=False)
        angular = GeometryBridge.avstack_to_angular_vel(
            obj_state.angular_velocity, stamped=False
        )
        pose = Pose(position=position, orientation=orientation)
        twist = Twist(linear=linear, angular=angular)
        state = ObjectState(
            obj_type=obj_state.obj_type if obj_state.obj_type else "",
            pose=pose,
            twist=twist,
            linear_acceleration=GeometryBridge.avstack_to_acceleration(
                obj_state.acceleration, stamped=False
            ),
            angular_acceleration=Vector3(),  # TODO: we don't have this in avstack
            box=GeometryBridge.avstack_to_box3d(obj_state.box, stamped=False),
        )
        return state

    @classmethod
    def avstack_to_objectstatestamped(
        cls, obj_state: ObjectStateAV, header=None
    ) -> ObjectStateStamped:
        if not header:
            header = Bridge.reference_to_header(obj_state.reference)
        state = cls.avstack_to_objectstate(obj_state)
        return ObjectStateStamped(header=header, state=state)

    @classmethod
    def avstack_to_objecstatearray(
        cls,
        obj_states: List[ObjectStateAV],
        header=None,
    ) -> ObjectStateArray:
        if len(obj_states) == 0:
            if not header:
                raise NotImplementedError("How do we get header without objects?")
            return ObjectStateArray(header=header)
        if not header:
            header = Bridge.reference_to_header(obj_states[0].reference)
        states = [cls.avstack_to_objectstate(obj) for obj in obj_states]
        return ObjectStateArray(header=header, states=states)
