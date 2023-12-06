import numpy as np
from geometry_msgs.msg import TransformStamped, Twist, Vector3Stamped, Quaternion
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from vision_msgs.msg import BoundingBox3D

from avstack_msgs.msg import BoxTrack, BoxTrackStamped, ObjectState, ObjectStateStamped
from .base import Bridge


def do_transform_object_state_stamped(
    obj_state: ObjectStateStamped, tf: TransformStamped
) -> ObjectStateStamped:
    obj_state_tf = ObjectStateStamped(
        state=do_transform_object_state(obj_state.state, tf)
    )
    obj_state_tf.header = tf.header
    return obj_state_tf


def do_transform_object_state(
    obj_state: ObjectState, tf: TransformStamped
) -> ObjectState:
    """Apply transform to object state

    NOTE: the vector3 transform operates on stamped vectors but does not
    actually use the header, so we can safely wrap the transformation
    """
    pose = do_transform_pose(obj_state.pose, tf)
    linear_vel = do_transform_vector3(
        Vector3Stamped(vector=obj_state.twist.linear), tf
    ).vector
    angular_vel = do_transform_vector3(
        Vector3Stamped(vector=obj_state.twist.angular), tf
    ).vector
    linear_acc = do_transform_vector3(
        Vector3Stamped(vector=obj_state.linear_acceleration), tf
    ).vector
    angular_acc = do_transform_vector3(
        Vector3Stamped(vector=obj_state.angular_acceleration), tf
    ).vector
    box = do_transform_box(obj_state.box, tf)

    obj_state_tf = ObjectState(
        obj_type=obj_state.obj_type,
        pose=pose,
        twist=Twist(linear=linear_vel, angular=angular_vel),
        linear_acceleration=linear_acc,
        angular_acceleration=angular_acc,
        box=box,
    )

    return obj_state_tf


def do_transform_box_track_stamped(
    box_track: BoxTrackStamped, tf: TransformStamped
) -> BoxTrackStamped:
    box_track_tf = BoxTrackStamped(
        track=do_transform_box_track(box_track.track, tf)
    )
    box_track_tf.header = tf.header
    return box_track_tf


def do_transform_box_track(
    track: BoxTrack, tf: TransformStamped
) -> BoxTrack:
    """Apply transform to box track
    
    NOTE: the vector3 transform note above applies
    
    We also need to transform the covariance matrix
    """
    p_tf = Bridge.ndarray_to_list(
        do_transform_box_track_covariance(Bridge.list_to_2d_ndarray(track.p), tf)
    )
    velocity = do_transform_vector3(Vector3Stamped(vector=track.velocity), tf).vector
    box_track_tf = BoxTrack(
        obj_type=track.obj_type,
        box=do_transform_box(track.box, tf),
        velocity=velocity,
        p=p_tf,
        n_updates=track.n_updates,
        age=track.age,
        coast=track.coast,
        identifier=track.identifier,
    )

    return box_track_tf


def do_transform_box(box: BoundingBox3D, tf: TransformStamped) -> BoundingBox3D:
    center = do_transform_pose(box.center, tf)
    box_tf = BoundingBox3D(center=center, size=box.size)
    return box_tf


def do_transform_box_track_covariance(cov_in: np.ndarray, tf: TransformStamped) -> np.ndarray:
    """Apply the transform to the box track covariance matrix
    
    The covariance is a 9x9 where the state vector is:
    [x, y, z, h, w, l, vx, vy, vz]
    """
    assert cov_in.shape == (9,9), cov_in.shape
    zero = np.zeros((3,3))
    eye = np.eye(3)
    R = _quat_to_rot(tf.transform.rotation)
    R_block = np.block([[   R, zero, zero],
                        [zero,  eye, zero],
                        [zero, zero,    R]])
    cov_out = R_block @ cov_in @ R_block.T
    return cov_out


def _quat_to_rot(tf_rot: Quaternion):
    # Converting the Quaternion to a Rotation Matrix first
    # Taken from: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    q0 = tf_rot.w
    q1 = tf_rot.x
    q2 = tf_rot.y
    q3 = tf_rot.z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # Code reference: https://github.com/ros2/geometry2/pull/430
    # Mathematical Reference:
    # A. L. Garcia, “Linear Transformations of Random Vectors,” in Probability,
    # Statistics, and Random Processes For Electrical Engineering, 3rd ed.,
    # Pearson Prentice Hall, 2008, pp. 320–322.

    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    return R