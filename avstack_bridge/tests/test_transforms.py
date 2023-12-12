import numpy as np
from avstack.geometry import PassiveReferenceFrame, q_mult_vec, transform_orientation
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
    Vector3Stamped,
)
from std_msgs.msg import Header

from avstack_bridge import base, transform


t0 = 0.0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)
header_world = base.Bridge.reference_to_header(passive_world_reference)
header_agent = base.Bridge.reference_to_header(passive_agent_reference)


"""
Testing basic ROS2 implementations against straight math
"""


def test_do_transform_point_translation():
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    # NOTE: applying a ros2 transform is a dx -- the opposite of our ref frame
    dx = np.array([0, 4, 8], dtype=float)
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    point = Point(x=x0[0], y=x0[1], z=x0[2])
    point_world_ros = PointStamped(header=header_world, point=point)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion()
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    point_agent_ros = transform.do_transform_point(point_world_ros, tf_world_to_agent)
    # NOTE: applying a ros2 transform is a dx
    xf = x0 + dx
    assert np.isclose(point_agent_ros.point.x, xf[0])
    assert np.isclose(point_agent_ros.point.y, xf[1])
    assert np.isclose(point_agent_ros.point.z, xf[2])


def test_do_transform_point_rotation():
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    # NOTE: applying a ros2 transform is a dq -- the opposite of our ref frame
    dq = transform_orientation([np.pi / 6, np.pi / 4, np.pi / 2], "euler", "quat")
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    point = Point(x=x0[0], y=x0[1], z=x0[2])
    point_world_ros = PointStamped(header=header_world, point=point)
    tf_translation = Vector3()
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    point_agent_ros = transform.do_transform_point(point_world_ros, tf_world_to_agent)
    # NOTE: applying a ros2 transform is a dq
    xf = q_mult_vec(dq, x0)
    assert np.isclose(point_agent_ros.point.x, xf[0])
    assert np.isclose(point_agent_ros.point.y, xf[1])
    assert np.isclose(point_agent_ros.point.z, xf[2])


def test_do_transform_vector_translation():
    """NOTE: vectors cannot be translated"""
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    dx = np.array([0, 4, 8], dtype=float)
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    vec = Vector3(x=x0[0], y=x0[1], z=x0[2])
    vec_world_ros = Vector3Stamped(header=header_world, vector=vec)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion()
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    vec_agent_ros = transform.do_transform_vector3(vec_world_ros, tf_world_to_agent)
    xf = x0
    assert np.isclose(vec_agent_ros.vector.x, xf[0])
    assert np.isclose(vec_agent_ros.vector.y, xf[1])
    assert np.isclose(vec_agent_ros.vector.z, xf[2])


def test_do_transform_point_full():
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    # NOTE: applying a ros2 transform is a dx -- the opposite of our ref frame
    dx = np.array([0, 4, 8], dtype=float)
    dq = transform_orientation([np.pi / 6, np.pi / 4, np.pi / 2], "euler", "quat")
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    point = Point(x=x0[0], y=x0[1], z=x0[2])
    point_world_ros = PointStamped(header=header_world, point=point)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    point_agent_ros = transform.do_transform_point(point_world_ros, tf_world_to_agent)
    # NOTE: applying a ros2 transform is basically the inverse of -dx and a -dq
    xf = q_mult_vec(dq, x0) + dx
    assert np.isclose(point_agent_ros.point.x, xf[0])
    assert np.isclose(point_agent_ros.point.y, xf[1])
    assert np.isclose(point_agent_ros.point.z, xf[2])


def test_do_transform_vector_full():
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    dx = np.array([1, 1, 1], dtype=float)
    dq = transform_orientation([0, 0, np.pi / 2], "euler", "quat")
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    vec = Vector3(x=x0[0], y=x0[1], z=x0[2])
    vec_world_ros = Vector3Stamped(header=header_world, vector=vec)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    vec_agent_ros = transform.do_transform_vector3(vec_world_ros, tf_world_to_agent)
    xf = q_mult_vec(dq, x0)
    assert np.isclose(vec_agent_ros.vector.x, xf[0])
    assert np.isclose(vec_agent_ros.vector.y, xf[1])
    assert np.isclose(vec_agent_ros.vector.z, xf[2])


def test_do_transform_pose_full():
    t0 = 0.0
    x0 = np.array([1, 2, 3], dtype=float)
    q0 = transform_orientation([np.pi / 4, np.pi / 2, np.pi / 6], "euler", "quat")
    # NOTE: applying a ros2 transform is a dx -- the opposite of our ref frame
    dx = np.array([0, 4, 8], dtype=float)
    dq = transform_orientation([np.pi / 6, np.pi / 4, np.pi / 2], "euler", "quat")
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    point = Point(x=x0[0], y=x0[1], z=x0[2])
    orient = Quaternion(x=q0.x, y=q0.y, z=q0.z, w=q0.w)
    pose = Pose(position=point, orientation=orient)
    # pose_world_ros = PoseStamped(header=header_world, pose=pose)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_world_to_agent = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    pose_agent_ros = transform.do_transform_pose(pose, tf_world_to_agent)
    # NOTE: applying a ros2 transform is basically the inverse of -dx and a -dq
    xf = q_mult_vec(dq, x0) + dx
    qf = dq * q0
    assert np.isclose(pose_agent_ros.position.x, xf[0])
    assert np.isclose(pose_agent_ros.position.y, xf[1])
    assert np.isclose(pose_agent_ros.position.z, xf[2])
    assert np.isclose(pose_agent_ros.orientation.x, qf.x)
    assert np.isclose(pose_agent_ros.orientation.y, qf.y)
    assert np.isclose(pose_agent_ros.orientation.z, qf.z)
    assert np.isclose(pose_agent_ros.orientation.w, qf.w)


"""
Testing basic ROS2 implementations against AVstack
"""


def test_do_transform_point_vs_avstack():
    pass


def test_do_transform_pose_vs_avstack():
    pass


"""
Testing custom transform implementations
"""


# def test_do_transform_point_no_quaternion():
#     # get active reference frame
#     active_agent_reference = ReferenceFrame(
#         x=np.array([1,2,3]),
#         q=np.quaternion(1),
#         reference=GlobalOrigin3D,
#         from_frame="world",
#         to_frame="agent",
#         timestamp=t0,
#     )
#     active_world_reference = GlobalOrigin3D
#     active_world_reference.timestamp = t0
#     tf_world_to_agent = base.Bridge.reference_to_tf2_stamped(active_agent_reference)

#     # define position in world frame
#     pos_in_world_passive = Position(np.array([1,1,1]), reference=passive_world_reference)
#     pos_in_world_active = Position(pos_in_world_passive.x, reference=active_world_reference)

#     # transform point via bridge transforms
#     point_in_world = geometry.GeometryBridge.avstack_to_position(pos_in_world_passive, stamped=True)
#     point_in_agent = transform.do_transform_point(point_in_world, transform=tf_world_to_agent)
#     position_via_bridge = geometry.GeometryBridge.position_to_avstack(point_in_agent.point, header=point_in_agent.header)

#     # transform point via avstack
#     position_via_avstack = pos_in_world_active.change_reference(active_agent_reference, inplace=False)

#     # check results
#     assert np.allclose(position_via_avstack.x, position_via_bridge.x)


# def test_do_transform_object_state():
#     obj_world_msg = objects.ObjectStateBridge.avstack_to_objectstatestamped(
#         obj_in_world_passive
#     )
#     assert obj_world_msg.header.frame_id == "world"

#     obj_in_world_to_agent_msg = transform.do_transform_objectstatestamped(
#         obj_world_msg, tf=tf_world_to_agent
#     )
#     assert obj_in_world_to_agent_msg.header.frame_id == "agent"

#     obj_in_world_to_agent = objects.ObjectStateBridge.objectstate_to_avstack(
#         obj_in_world_to_agent_msg
#     )
#     assert np.allclose(obj_in_world_to_agent.position.x, obj_in_agent_active.position.x)


# def test_do_transform_box_track():
#     raise


# def test_do_transform_box():
#     raise


# def test_do_transform_box_track_covariance():
#     raise


# def test_quat_to_rot():
#     raise
