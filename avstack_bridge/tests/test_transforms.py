import numpy as np
import PyKDL
from avstack.calibration import LidarCalibration
from avstack.geometry import (
    Attitude,
    GlobalOrigin3D,
    PassiveReferenceFrame,
    Position,
    ReferenceFrame,
    q_mult_vec,
    transform_orientation,
)
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
from tf2_ros.buffer import Buffer
from tf_utilities import (
    frame_to_matrix,
    pose_to_matrix,
    random_quat,
    random_tf,
    transform_to_kdl,
)
from utilities import get_box_3d, get_boxtrack_3d, get_point_cloud

from avstack_bridge import base, geometry, tracks, transform
from avstack_bridge.geometry import GeometryBridge
from avstack_bridge.sensors import LidarSensorBridge


t0 = 0.0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)
header_world = base.Bridge.reference_to_header(passive_world_reference)
header_agent = base.Bridge.reference_to_header(passive_agent_reference)


"""
Testing basic transform inversions
"""


def test_invert_tf():
    tf = random_tf()
    tf_inv = transform.invert_transform(tf)
    tf_inv_inv = transform.invert_transform(tf_inv)
    assert np.allclose(
        frame_to_matrix(transform_to_kdl(tf)),
        frame_to_matrix(transform_to_kdl(tf_inv_inv)),
    )


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


def test_do_transform_point():
    xp = np.array([1, 2, 3], dtype=float)

    # conversion with AVstack
    agent_reference = ReferenceFrame(
        x=np.random.rand(3),
        q=random_quat(),
        reference=GlobalOrigin3D,
        from_frame="world",
        to_frame="agent",
        timestamp=t0,
    )
    xp_avstack = Position(xp, reference=GlobalOrigin3D)
    xp_avstack_new = xp_avstack.change_reference(agent_reference, inplace=False)

    # conversion with ros
    agent_tf_frame = base.Bridge.reference_to_tf2_stamped(agent_reference)
    agent_tf_data = transform.invert_transform(agent_tf_frame)
    xp_ros = PointStamped(point=Point(x=xp[0], y=xp[1], z=xp[2]))
    xp_ros_new = transform.do_transform_point(xp_ros, agent_tf_data).point
    xp_ros_new = np.array([xp_ros_new.x, xp_ros_new.y, xp_ros_new.z])

    assert np.allclose(xp_avstack_new.x, xp_ros_new)


def test_do_transform_pose():
    """NOTE the definition of a ros pose differs from avstack's"""
    xp = np.array([1, 2, 3], dtype=float)
    qp = random_quat()
    xp_avstack_active = Position(xp, reference=GlobalOrigin3D)
    qp_avstack_active = Attitude(qp, reference=GlobalOrigin3D)
    xp_avstack_passive = Position(xp, reference=GlobalOrigin3D.as_passive_frame())
    qp_avstack_passive = Attitude(qp, reference=GlobalOrigin3D.as_passive_frame())

    # conversion with AVstack
    agent_reference = ReferenceFrame(
        x=np.random.rand(3),
        q=random_quat(),
        reference=GlobalOrigin3D,
        from_frame="world",
        to_frame="agent",
        timestamp=t0,
    )
    xp_avstack_new = xp_avstack_active.change_reference(agent_reference, inplace=False)
    qp_avstack_new = qp_avstack_active.change_reference(agent_reference, inplace=False)
    xp_avstack_new_passive = Position(
        xp_avstack_new.x, reference=agent_reference.as_passive_frame()
    )
    qp_avstack_new_passive = Attitude(
        qp_avstack_new.q, reference=agent_reference.as_passive_frame()
    )
    pose_avstack_new = geometry.GeometryBridge.avstack_to_pose(
        pos=xp_avstack_new_passive, att=qp_avstack_new_passive, stamped=True
    )
    xp_ros_new_avstack, qp_ros_new_avstack = geometry.GeometryBridge.pose_to_avstack(
        pose=pose_avstack_new.pose, header=pose_avstack_new.header
    )

    # conversion with ros
    agent_tf_frame = base.Bridge.reference_to_tf2_stamped(agent_reference)
    agent_tf_data = transform.invert_transform(agent_tf_frame)
    pose_ros = geometry.GeometryBridge.avstack_to_pose(
        pos=xp_avstack_passive, att=qp_avstack_passive, stamped=True
    )
    pose_ros_new = transform.do_transform_pose(pose_ros.pose, agent_tf_data)
    xp_ros_new, qp_ros_new = geometry.GeometryBridge.pose_to_avstack(
        pose=pose_ros_new, header=agent_tf_data.header
    )

    assert np.allclose(xp_ros_new_avstack.x, xp_ros_new.x)
    assert np.allclose(
        (qp_ros_new_avstack.q * qp_ros_new.q.conjugate()).vec, np.zeros(3)
    )


"""
Testing custom transform implementations
"""


def test_do_transform_cloud():
    pc = get_point_cloud(seed=1)
    tf_frame = random_tf(translation=True, rotation=True, seed=1)
    tf_data = transform.invert_transform(tf_frame)
    ref2 = base.Bridge.tf2_to_reference(tf_frame)

    # apply transform with ros
    pc2_ros = LidarSensorBridge.avstack_to_pc2(pc, header=tf_frame.header)
    pc2_ros_tf = transform.do_transform_cloud(pc2_ros, tf_data)

    # apply transform with avstack
    calib2 = LidarCalibration(reference=ref2)
    pc_avstack_tf = pc.project(calib2)
    pc_ros_tf_avstack = LidarSensorBridge.pc2_to_avstack(pc2_ros_tf)

    # check equivalence
    assert np.allclose(pc_avstack_tf.data.x, pc_ros_tf_avstack.data.x)


def test_do_transform_box_translate():
    box = get_box_3d(seed=1)
    tf_frame = random_tf(translation=True, rotation=True, seed=1)
    tf_data = transform.invert_transform(tf_frame)
    ref2 = base.Bridge.tf2_to_reference(tf_frame)

    # apply transform with ros
    box_ros = GeometryBridge.avstack_to_box3d(box, stamped=False)
    box_ros_tf = transform.do_transform_box(box_ros, tf=tf_data)
    box_ros_tf_avstack = GeometryBridge.box3d_to_avstack(
        box_ros_tf, header=tf_data.header
    )

    # apply transform with avstack
    box_avstack_tf = box.change_reference(ref2, inplace=False)

    # check equivalence
    assert np.allclose(box_ros_tf_avstack.size, box_avstack_tf.size)
    assert np.allclose(box_ros_tf_avstack.center.x, box_avstack_tf.center.x)
    assert np.allclose(box_ros_tf_avstack.attitude.q, box_avstack_tf.attitude.q)


def test_do_transform_BoxTrack3D():
    box_track = get_boxtrack_3d(seed=1)
    tf_frame = random_tf(translation=True, rotation=True, seed=1)
    tf_data = transform.invert_transform(tf_frame)
    ref2 = base.Bridge.tf2_to_reference(tf_frame)

    # apply transformation with ros
    box_track_ros = tracks.TrackBridge.avstack_to_BoxTrack3D(
        box_track, header=tf_frame.header
    )
    box_track_ros_tf = transform.do_transform_BoxTrack3D(box_track_ros, tf_data)
    box_track_ros_tf_avstack = tracks.TrackBridge.boxtrack_to_avstack(box_track_ros_tf)

    # apply transformation with avstack
    box_track_avstack_tf = box_track.change_reference(ref2, inplace=False)

    # check equivalence
    assert np.allclose(
        box_track_ros_tf_avstack.box.center.x, box_track_avstack_tf.box.center.x
    )
    assert np.allclose(
        box_track_ros_tf_avstack.box.attitude.q, box_track_avstack_tf.box.attitude.q
    )


"""
Testing custom transform implementations
"""


def test_buffer():
    # set tf frame
    tf_frame = random_tf(translation=True, rotation=True, seed=1)
    tf_buffer = Buffer()
    tf_buffer.set_transform(tf_frame, authority="test")

    # check original frame
    tf_frame_lookup = tf_buffer.lookup_transform(
        target_frame=tf_frame.header.frame_id,  # will be "world"
        source_frame=tf_frame.child_frame_id,  # will be "agent"
        time=tf_frame.header.stamp,
    )
    assert tf_frame.header.frame_id == tf_frame_lookup.header.frame_id == "world"
    assert tf_frame.child_frame_id == tf_frame_lookup.child_frame_id == "agent"
    for attr in ["x", "y", "z"]:
        assert np.isclose(
            getattr(tf_frame.transform.translation, attr),
            getattr(tf_frame_lookup.transform.translation, attr),
        )

    # check frame inversion
    tf_data = transform.invert_transform(tf_frame)
    tf_data_lookup = tf_buffer.lookup_transform(
        target_frame=tf_frame.child_frame_id,  # will be "agent" -- data to be transform here
        source_frame=tf_frame.header.frame_id,  # will be "world" -- data originated here
        time=tf_frame.header.stamp,
    )
    assert tf_data.header.frame_id == tf_data_lookup.header.frame_id == "agent"
    assert tf_data.child_frame_id == tf_data_lookup.child_frame_id == "world"
    for attr in ["x", "y", "z"]:
        assert np.isclose(
            getattr(tf_data.transform.translation, attr),
            getattr(tf_data_lookup.transform.translation, attr),
        )
