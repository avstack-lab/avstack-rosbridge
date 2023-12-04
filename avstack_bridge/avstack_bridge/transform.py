from geometry_msgs.msg import TransformStamped, Twist
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from vision_msgs.msg import BoundingBox3D

from avstack_msgs.msg import ObjectState


def do_transform_object_state(
    obj_state: ObjectState, tf: TransformStamped
) -> ObjectState:
    obj_type = obj_state.obj_type
    pose = do_transform_pose(obj_state.pose, tf)
    linear_vel = do_transform_vector3(obj_state.twist.linear, tf)
    angular_vel = do_transform_vector3(obj_state.twist.angular, tf)
    linear_acc = do_transform_vector3(obj_state.linear_acceleration, tf)
    angular_acc = do_transform_vector3(obj_state.angular_acceleration, tf)
    box = do_transform_box(obj_state.box, tf)

    obj_state_tf = ObjectState(
        obj_type=obj_state.obj_type,
        pose=pose,
        twist=Twist(linear=linear_vel, angular=angular_vel),
        linear_acceleration=linear_acc,
        angular_acc=angular_acc,
        box=box,
    )

    return obj_state_tf


def do_transform_box(box: BoundingBox3D, tf: TransformStamped) -> BoundingBox3D:
    center = do_transform_pose(box.center, tf)
    box_tf = BoundingBox3D(center=center, size=box.size)
    return box_tf
