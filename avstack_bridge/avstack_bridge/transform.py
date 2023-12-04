from geometry_msgs.msg import TransformStamped, Twist, Vector3Stamped
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from vision_msgs.msg import BoundingBox3D

from avstack_msgs.msg import ObjectState, ObjectStateStamped


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


def do_transform_box(box: BoundingBox3D, tf: TransformStamped) -> BoundingBox3D:
    center = do_transform_pose(box.center, tf)
    box_tf = BoundingBox3D(center=center, size=box.size)
    return box_tf
