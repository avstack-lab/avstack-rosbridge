import numpy as np
from avstack.geometry import (
    Attitude,
    GlobalOrigin3D,
    PassiveReferenceFrame,
    Position,
    ReferenceFrame,
)
from utilities import get_object_global

from avstack_bridge import base, geometry, objects, transform


t0 = 0.0

active_agent_reference = ReferenceFrame(
    x=np.random.randn(3),
    q=np.quaternion(*np.random.rand(3)),
    reference=GlobalOrigin3D,
    from_frame="world",
    to_frame="agent",
    timestamp=t0,
)
active_world_reference = GlobalOrigin3D
active_world_reference.timestamp = t0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)

header_world = base.Bridge.reference_to_header(passive_world_reference)
header_agent = base.Bridge.reference_to_header(passive_agent_reference)
tf_world_to_agent = base.Bridge.reference_to_tf2_stamped(active_agent_reference)

obj_in_world_passive = get_object_global(seed=1, reference=passive_world_reference)
obj_in_world_active = get_object_global(seed=1, reference=active_world_reference)
obj_in_agent_active = obj_in_world_active.change_reference(
    reference=active_agent_reference, inplace=False
)
# obj_in_agent = get_object_global(seed=1, reference=active_agent_reference)


def test_do_transform_pose():
    position = Position(np.random.randn(3), passive_world_reference)
    attitude = Attitude(np.quaternion(*np.random.randn(3)), passive_world_reference)
    position_active = Position(position.x, active_world_reference)
    attitude_active = Attitude(attitude.q, active_world_reference)

    pose_in_world = geometry.GeometryBridge.avstack_to_pose(
        pos=position, att=attitude, stamped=False
    )
    pose_in_agent = transform.do_transform_pose(
        pose=pose_in_world, transform=tf_world_to_agent
    )

    position_in_agent_avstack = position_active.change_reference(
        active_agent_reference, inplace=False
    )
    attitude_in_agent_avstack = attitude_active.change_reference(
        active_agent_reference, inplace=False
    )

    (
        position_in_agent_avstack_2,
        attitude_in_agent_avstack_2,
    ) = geometry.GeometryBridge.pose_to_avstack(pose_in_agent, header=header_agent)
    assert np.allclose(
        position_in_agent_avstack.position.x, position_in_agent_avstack_2.x
    )


def test_do_transform_object_state():
    obj_world_msg = objects.ObjectStateBridge.avstack_to_objectstatestamped(
        obj_in_world_passive
    )
    assert obj_world_msg.header.frame_id == "world"

    obj_in_world_to_agent_msg = transform.do_transform_objectstatestamped(
        obj_world_msg, tf=tf_world_to_agent
    )
    assert obj_in_world_to_agent_msg.header.frame_id == "agent"

    obj_in_world_to_agent = objects.ObjectStateBridge.objectstate_to_avstack(
        obj_in_world_to_agent_msg
    )
    assert np.allclose(obj_in_world_to_agent.position.x, obj_in_agent_active.position.x)


# def test_do_transform_box_track():
#     raise


# def test_do_transform_box():
#     raise


# def test_do_transform_box_track_covariance():
#     raise


# def test_quat_to_rot():
#     raise
