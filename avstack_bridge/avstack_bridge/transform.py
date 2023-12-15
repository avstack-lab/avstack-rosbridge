"""
Reference frames are always confusing. Here is a description of how they
work in TF2 in my understanding:

A transform in ROS is a way of representing a passive transformation
that operates on the reference frame origins themselves. A transform
with frame_id (source) A and child_frame_id (target) B specifies the transformation
that will take the source frame to the target frame.

To take a point in frame A to its corresponding point in frame B,
you must get the active transform which is the inverse -- the
transform from source B to target A. 

https://answers.ros.org/question/194046/the-problem-of-transformerlookuptransform/
https://answers.ros.org/question/229463/confusing-tf-transforms/
https://robotics.stackexchange.com/questions/97873/default-transform-direction-if-tf2


See: https://github.com/ros2/geometry2/tree/humble/tf2_geometry_msgs
"""

import numpy as np
import tf2_ros
from avstack.geometry import q_mult_vec
from geometry_msgs.msg import (
    Quaternion,
    Transform,
    TransformStamped,
    TwistStamped,
    Vector3,
    Vector3Stamped,
)
from std_msgs.msg import Header
from tf2_geometry_msgs import (  # noqa
    do_transform_point,
    do_transform_pose,
    do_transform_vector3,
)
from vision_msgs.msg import BoundingBox3D

from avstack_bridge.base import Bridge
from avstack_msgs.msg import BoxTrack, BoxTrackStamped, ObjectStateStamped


def do_transform_twist(
    twist: TwistStamped, transform: TransformStamped
) -> TwistStamped:
    res = TwistStamped()
    res.twist.linear = do_transform_vector3(
        Vector3Stamped(header=twist.header, vector=twist.linear), transform
    )
    res.twist.angular = do_transform_vector3(
        Vector3Stamped(header=twist.header, vector=twist.angular), transform
    )
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(TwistStamped, do_transform_twist)


def do_transform_objectstate(
    objectstate: ObjectStateStamped, transform: TransformStamped
) -> ObjectStateStamped:
    res = ObjectStateStamped()
    res.state.obj_type = objectstate.state.obj_type
    res.state.pose = do_transform_pose(objectstate.state.pose, transform)
    res.state.twist = do_transform_twist(
        TwistStamped(header=objectstate.header, twist=objectstate.state.twist),
        transform,
    )
    res.state.linear_acceleration = do_transform_vector3(
        Vector3Stamped(
            header=objectstate.header, vector=objectstate.state.linear_acceleration
        ),
        transform,
    )
    res.state.angular_acceleration = do_transform_vector3(
        Vector3Stamped(
            header=objectstate.header, vector=objectstate.state.angular_acceleration
        ),
        transform,
    )
    res.state.box = do_transform_box(objectstate.state.box, transform)
    res.header = transform.header
    return res


tf2_ros.TransformRegistration().add(ObjectStateStamped, do_transform_objectstate)


def do_transform_box(box: BoundingBox3D, tf: TransformStamped) -> BoundingBox3D:
    center = do_transform_pose(box.center, tf)
    box_tf = BoundingBox3D(center=center, size=box.size)
    return box_tf


tf2_ros.TransformRegistration().add(BoundingBox3D, do_transform_box)


def do_transform_boxtrack(
    boxtrack: BoxTrackStamped, tf: TransformStamped
) -> BoxTrackStamped:
    track = boxtrack.track
    p_tf = Bridge.ndarray_to_list(
        do_transform_boxtrack_covariance(Bridge.list_to_2d_ndarray(track.p), tf)
    )
    velocity = do_transform_vector3(Vector3Stamped(vector=track.velocity), tf).vector
    track_tf = BoxTrack(
        obj_type=track.obj_type,
        box=do_transform_box(track.box, tf),
        velocity=velocity,
        p=p_tf,
        n_updates=track.n_updates,
        age=track.age,
        coast=track.coast,
        identifier=track.identifier,
    )

    boxtrack_tf = BoxTrackStamped()
    boxtrack_tf.track = track_tf
    boxtrack_tf.header = tf.header

    return boxtrack_tf


tf2_ros.TransformRegistration().add(BoxTrackStamped, do_transform_boxtrack)


def do_transform_boxtrack_covariance(
    cov_in: np.ndarray, tf: TransformStamped
) -> np.ndarray:
    """Apply the transform to the box track covariance matrix

    The covariance is a 9x9 where the state vector is:
    [x, y, z, h, w, l, vx, vy, vz]
    """
    assert cov_in.shape == (9, 9), cov_in.shape
    zero = np.zeros((3, 3))
    eye = np.eye(3)

    R = _get_rotmat_from_tf(tf.transform)
    R_block = np.block([[R, zero, zero], [zero, eye, zero], [zero, zero, R]])
    cov_out = R_block @ cov_in @ R_block.T
    return cov_out


def _get_rotmat_from_tf(transform: TransformStamped) -> np.ndarray:
    return _get_mat_from_quat(_get_quat_from_tf(transform))


def _get_quat_from_tf(transform: TransformStamped) -> np.ndarray:
    """Get the np array for the quaternion in wxyz"""
    quat = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ]
    return np.array(quat)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.

    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [
            [1.0 - (yYZ[0] + zZ), xXYZ[1] - wXYZ[2], xXYZ[2] + wXYZ[1]],
            [xXYZ[1] + wXYZ[2], 1.0 - (xXYZ[0] + zZ), yYZ[1] - wXYZ[0]],
            [xXYZ[2] - wXYZ[1], yYZ[1] + wXYZ[0], 1.0 - (xXYZ[0] + yYZ[0])],
        ]
    )


def invert_transform(tf: TransformStamped):
    """Invert the transform"""
    q = np.quaternion(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
    )
    t = np.array(
        [
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ]
    )
    q2 = q.conjugate()
    t2 = -q_mult_vec(q2, t)
    rotation = Quaternion(x=q2.x, y=q2.y, z=q2.z, w=q2.w)
    translation = Vector3(x=t2[0], y=t2[1], z=t2[2])
    header = Header(frame_id=tf.child_frame_id, stamp=tf.header.stamp)
    return TransformStamped(
        header=header,
        child_frame_id=tf.header.frame_id,
        transform=Transform(translation=translation, rotation=rotation),
    )
