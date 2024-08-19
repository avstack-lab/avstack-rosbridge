import numpy as np
import PyKDL
from avstack.geometry import PassiveReferenceFrame, transform_orientation
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3

from avstack_bridge.base import Bridge


t0 = 0.0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)
header_world = Bridge.reference_to_header(passive_world_reference)
header_agent = Bridge.reference_to_header(passive_agent_reference)


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


def random_tf(translation: bool = True, rotation: bool = True, seed: int = None):
    if seed:
        np.random.seed(seed)
    dx = np.random.rand(3) if translation else np.zeros((3,))
    dq = random_quat() if rotation else np.quaternion(1)
    tf_translation = Vector3(x=dx[0], y=dx[1], z=dx[2])
    tf_rotation = Quaternion(x=dq.x, y=dq.y, z=dq.z, w=dq.w)
    tf = Transform(translation=tf_translation, rotation=tf_rotation)
    tf_stamped = TransformStamped(
        header=header_world, child_frame_id="agent", transform=tf
    )
    return tf_stamped


def random_quat() -> np.quaternion:
    return transform_orientation(np.random.rand(3), "euler", "quat")
