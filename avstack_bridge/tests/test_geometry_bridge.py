import numpy as np
import quaternion  # noqa # pylint: disable=unused-import
from avstack.geometry import Attitude, GlobalOrigin3D, Position
from utilities import random_quat

from avstack_bridge.geometry import GeometryBridge


def test_pose_bridge():
    xp = np.random.randn(3)
    qp = random_quat()

    position = Position(x=xp, reference=GlobalOrigin3D.as_passive_frame())
    attitude = Attitude(q=qp, reference=GlobalOrigin3D.as_passive_frame())
    pose = GeometryBridge.avstack_to_pose(position, attitude, stamped=True)
    position_2, attitude_2 = GeometryBridge.pose_to_avstack(
        pose.pose, header=pose.header
    )

    assert position.allclose(position_2)
    assert attitude.allclose(attitude_2)
