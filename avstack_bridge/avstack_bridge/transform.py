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
And: https://docs.ros.org/en/ros2_packages/rolling/api/tf2_ros/generated/classtf2__ros_1_1BufferInterface.html
"""
from typing import Union
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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_geometry_msgs import (  # noqa
    do_transform_point,
    do_transform_pose,
    do_transform_vector3,
)
from vision_msgs.msg import BoundingBox3D
try:
    from numpy.lib.recfunctions import (structured_to_unstructured, unstructured_to_structured)
except ImportError:
    # Fix for RHEL because its NumPy version does not include these functions
    from sensor_msgs_py.numpy_compat import (structured_to_unstructured,
                                             unstructured_to_structured)
from sensor_msgs_py.point_cloud2 import create_cloud, read_points


from avstack_bridge.base import Bridge
from avstack_msgs.msg import BoxTrack, BoxTrackStamped, ObjectStateStamped



def to_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)


def from_msg_msg(msg):
    return msg


tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)


def transform_points(
        point_cloud: np.ndarray,
        transform: Transform) -> np.ndarray:
    """
    Transform a bulk of points from an numpy array using a provided `Transform`.

    :param point_cloud: nx3 Array of points where n is the number of points
    :param transform: TF2 transform used for the transformation
    :returns: Array with the same shape as the input array, but with the transformation applied
    """
    # Build affine transformation
    transform_translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ])
    transform_rotation_matrix = _get_mat_from_quat(
        np.array([
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z
        ]))

    # "Batched" matmul meaning a matmul for each point
    # First we offset all points by the translation part
    # followed by a rotation using the rotation matrix
    return np.einsum(
        'ij, pj -> pi',
        transform_rotation_matrix,
        point_cloud) + transform_translation


def do_transform_cloud(
        cloud: PointCloud2,
        transform: Union[Transform, TransformStamped]) -> PointCloud2:
    """
    Apply a `Transform` or `TransformStamped` on a `PointCloud2`.

    The x, y, and z values are transformed into a different frame,
    while the rest of the cloud is kept untouched.

    :param cloud: The point cloud that should be transformed
    :param transform: The transform which will applied to the point cloud
    :returns: The transformed point cloud
    """
    # Create new Header so original header is not altered
    new_header = Header(
        stamp=cloud.header.stamp,
        frame_id=cloud.header.frame_id)

    # Check if we have a TransformStamped and are able to update the frame_id
    if isinstance(transform, TransformStamped):
        new_header.frame_id = transform.header.frame_id
        transform = transform.transform

    # Check if xyz are a subset of the field names
    required_fields = set('xyz')
    present_fields = {field.name for field in cloud.fields}
    assert required_fields <= present_fields, \
        'Point cloud needs the fields x, y, and z for the transformation'

    # Read points as structured NumPy array
    points = read_points(cloud)

    # Transform xyz part of the pointcloud using the given transform
    transformed_xyz = transform_points(
        structured_to_unstructured(points[['x', 'y', 'z']]),
        transform)

    # Check if there are additional fields that need to be merged with the transformed coordinates
    if required_fields != present_fields:
        # Merge original array including non coordinate fields with the transformed coordinates
        # The copy is needed as the original message would be altered otherwise
        points = points.copy()
        points[['x', 'y', 'z']] = unstructured_to_structured(transformed_xyz)
    else:
        points = transformed_xyz

    # Serialize pointcloud message
    return create_cloud(new_header, cloud.fields, points)


tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)


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
    center_tf = do_transform_pose(box.center, tf)
    box_tf = BoundingBox3D(center=center_tf, size=box.size)
    return box_tf


tf2_ros.TransformRegistration().add(BoundingBox3D, do_transform_box)


def do_transform_boxtrack(
    boxtrack: BoxTrackStamped, tf: TransformStamped
) -> BoxTrackStamped:
    track = boxtrack.track
    p_tf = Bridge.ndarray_to_list(
        do_transform_boxtrack_covariance(Bridge.list_to_2d_ndarray(track.p), tf)
    )
    box_tf = do_transform_box(track.box, tf)
    v_tf = do_transform_vector3(Vector3Stamped(vector=track.velocity), tf).vector
    track_tf = BoxTrack(
        obj_type=track.obj_type,
        box=box_tf,
        velocity=v_tf,
        p=p_tf,
        n_updates=track.n_updates,
        dt_coast=track.dt_coast,
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
