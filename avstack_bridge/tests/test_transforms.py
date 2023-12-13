import numpy as np
import PyKDL
import quaternion
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

from avstack_bridge import base, geometry, transform


t0 = 0.0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)
header_world = base.Bridge.reference_to_header(passive_world_reference)
header_agent = base.Bridge.reference_to_header(passive_agent_reference)


def random_quat():
    return transform_orientation(np.random.rand(3), "euler", "quat")


def test_random_quat():
    q = random_quat()
    assert np.allclose(q, np.normalized(q))
    assert np.allclose(q.conjugate(), np.normalized(q.conjugate()))


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


def transform_to_kdl(t):
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ),
        PyKDL.Vector(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ),
    )


def frame_to_matrix(frame):
    T_frame = np.array(
        [
            [frame[(0, 0)], frame[(0, 1)], frame[(0, 2)], frame[(0, 3)]],
            [frame[(1, 0)], frame[(1, 1)], frame[(1, 2)], frame[(1, 3)]],
            [frame[(2, 0)], frame[(2, 1)], frame[(2, 2)], frame[(2, 3)]],
            [0, 0, 0, 1],
        ]
    )
    return T_frame


def pose_to_matrix(pose):
    p_vec = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
    tf = TransformStamped(
        transform=Transform(rotation=pose.orientation, translation=p_vec)
    )
    return frame_to_matrix(transform_to_kdl(tf))


def world_tf():
    tf_stamped = TransformStamped(
        header=header_world, child_frame_id="agent", transform=Transform()
    )
    return tf_stamped


def random_tf():
    dx = np.random.rand(3)
    dq = random_quat()
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_stamped = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    return tf_stamped


def test_frame_create():
    # -- tf to matrix
    tf = random_tf()
    q_world_to_agent = geometry.GeometryBridge.attitude_to_avstack(
        tf.transform.rotation, header=header_agent
    )
    x_world_to_agent = geometry.GeometryBridge.position_to_avstack(
        tf.transform.translation, header=header_agent
    )

    # -- tf to frame
    frame = transform_to_kdl(tf)
    T_frame = frame_to_matrix(frame)
    q_frame = transform_orientation(T_frame[:3, :3], "dcm", "quat")
    qxyzw = frame.M.GetQuaternion()
    q_frame_via_q = np.quaternion(qxyzw[3], *qxyzw[:3])
    x_frame = T_frame[:3, 3]

    assert quaternion.allclose(q_frame, q_frame_via_q)
    assert np.allclose(q_world_to_agent.q, q_frame)
    assert np.allclose(x_world_to_agent.x, x_frame)


def test_frame_compose():
    tf1, tf2 = random_tf(), random_tf()
    frame_w_to_1, frame_1_to_2 = transform_to_kdl(tf1), transform_to_kdl(tf2)

    # frame way
    frame_w_to_2 = frame_1_to_2 * frame_w_to_1
    M_w_to_2_fra = frame_to_matrix(frame_w_to_2)

    # matrix way
    M_w_to_1 = frame_to_matrix(frame_w_to_1)
    M_1_to_2 = frame_to_matrix(frame_1_to_2)
    M_w_to_2_mat = M_1_to_2 @ M_w_to_1

    assert np.allclose(M_w_to_2_fra, M_w_to_2_mat)


def test_frame_apply_point():
    tf = random_tf()
    frame = transform_to_kdl(tf)
    x0 = np.array([1, 2, 3], dtype=float)

    # apply with pykdl
    point_kdl = PyKDL.Vector(*x0)
    point_kdl_new = frame * point_kdl
    point_kdl_new = np.array([point_kdl_new.x(), point_kdl_new.y(), point_kdl_new.z()])

    # apply with ros
    header_world = Header(frame_id="world", stamp=base.Bridge.time_to_rostime(t0))
    point_ros = PointStamped(
        header=header_world, point=Point(x=x0[0], y=x0[1], z=x0[2])
    )
    point_ros_new = transform.do_transform_point(point_ros, tf).point
    point_ros_new = np.array([point_ros_new.x, point_ros_new.y, point_ros_new.z])

    # apply with matrix math
    point_np_new = (frame_to_matrix(frame) @ np.array([*x0, 1]))[:3]

    assert np.allclose(point_kdl_new, point_ros_new)
    assert np.allclose(point_ros_new, point_np_new)


def test_frame_apply_pose():
    tf = random_tf()
    frame = transform_to_kdl(tf)
    x0 = np.array([1, 2, 3], dtype=float)
    q0 = random_quat()
    M0 = transform_orientation(q0, "quat", "dcm")
    position = Point(x=x0[0], y=x0[1], z=x0[2])
    p_vec = Vector3(x=x0[0], y=x0[1], z=x0[2])
    orientation = Quaternion(x=q0.x, y=q0.y, z=q0.z, w=q0.w)
    pose_ros = Pose(position=position, orientation=orientation)
    pose_M = np.block([[M0, x0[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]])
    pose_as_tf = TransformStamped(
        transform=Transform(translation=p_vec, rotation=orientation)
    )
    pose_as_frame = transform_to_kdl(pose_as_tf)

    # apply with pykdl
    pose_new_kdl = frame_to_matrix(frame * pose_as_frame)

    # apply with ros
    pose_new_ros = pose_to_matrix(transform.do_transform_pose(pose_ros, tf))

    # apply with matrix math
    pose_new_mat = frame_to_matrix(frame) @ pose_M

    assert np.allclose(pose_new_kdl, pose_new_ros)
    assert np.allclose(pose_new_ros, pose_new_mat)


"""
Testing basic ROS2 implementations against AVstack
"""


def test_avstack_to_kdl():
    pass


# def test_do_transform_point_vs_avstack():
#     # conversion with avstack
#     active_agent_reference = ReferenceFrame(
#         x=np.random.rand(3),
#         q=random_quat(),
#         reference=GlobalOrigin3D,
#         from_frame="world",
#         to_frame="agent",
#         timestamp=t0,
#     )
#     position_world_av = Position(np.random.rand(3), reference=GlobalOrigin3D)
#     position_agent_av = position_world_av.change_reference(
#         active_agent_reference, inplace=False
#     )

#     # conversion with ROS2
#     position_world_ros = Point(
#         x=position_world_av.x[0], y=position_world_av.x[1], z=position_world_av.x[2]
#     )
#     tf_world_to_agent = base.Bridge.reference_to_tf2_stamped(active_agent_reference)
#     position_agent_ros = transform.do_transform_point(
#         PointStamped(header=header_world, point=position_world_ros), tf_world_to_agent
#     ).point

#     # check results
#     assert np.isclose(position_agent_av.x[0], position_agent_ros.x)
#     assert np.isclose(position_agent_av.x[1], position_agent_ros.y)
#     assert np.isclose(position_agent_av.x[2], position_agent_ros.z)


# def test_do_transform_pose_vs_avstack():
#     # conversion with avstack
#     active_agent_reference = ReferenceFrame(
#         x=np.random.rand(3),
#         q=random_quat(),
#         reference=GlobalOrigin3D,
#         from_frame="world",
#         to_frame="agent",
#         timestamp=t0,
#     )
#     position_world_av = Position(np.random.rand(3), reference=GlobalOrigin3D)
#     attitude_world_av = Attitude(random_quat(), reference=GlobalOrigin3D)
#     position_agent_av = position_world_av.change_reference(
#         active_agent_reference, inplace=False
#     )
#     attitude_agent_av = attitude_world_av.change_reference(
#         active_agent_reference, inplace=False
#     )

#     # conversion with ROS2
#     position_world_ros = Point(
#         x=position_world_av.x[0], y=position_world_av.x[1], z=position_world_av.x[2]
#     )
#     orientation_world_ros = Quaternion(
#         x=attitude_world_av.q.x, y=attitude_world_av.q.y, z=attitude_world_av.q.z, w=attitude_world_av.q.w
#     )
#     pose_world_ros = Pose(position=position_world_ros, orientation=orientation_world_ros)
#     tf_world_to_agent = base.Bridge.reference_to_tf2_stamped(active_agent_reference)
#     pose_agent_ros = transform.do_transform_pose(
#         pose_world_ros, tf_world_to_agent
#     )


#     # check results
#     x_ros, q_ros = geometry.GeometryBridge.pose_to_avstack(pose_agent_ros, header=header_agent)
#     assert np.allclose(x_ros.x, position_agent_av.x)
#     assert quaternion.allclose(q_ros.q, attitude_agent_av.q) or \
#            quaternion.allclose(-1*q_ros.q, attitude_agent_av.q, atol=0.0000001)

# assert np.isclose(position_agent_av.x[0], pose_agent_ros.position.x)
# assert np.isclose(position_agent_av.x[1], pose_agent_ros.position.y)
# assert np.isclose(position_agent_av.x[2], pose_agent_ros.position.z)
# assert np.isclose(attitude_agent_av.q.x,  pose_agent_ros.orientation.x)
# assert np.isclose(attitude_agent_av.q.y,  pose_agent_ros.orientation.y)
# assert np.isclose(attitude_agent_av.q.z,  pose_agent_ros.orientation.z)
# assert np.isclose(attitude_agent_av.q.w,  pose_agent_ros.orientation.w)

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
