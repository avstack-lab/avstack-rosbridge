import numpy as np
import quaternion  # noqa # pylint: disable=unused-import
from avstack.geometry import Attitude, GlobalOrigin3D, Polygon, Position
from geometry_msgs.msg import PolygonStamped as RosPolygonStamped
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


def test_polygon_bridge():
    # avstack --> ros
    poly = Polygon(
        boundary=np.random.rand(10, 2), reference=GlobalOrigin3D.as_passive_frame()
    )
    poly_ros = GeometryBridge.avstack_to_polygon(poly, stamped=True)
    assert isinstance(poly_ros, RosPolygonStamped)

    # ros --> avstack
    poly_2 = GeometryBridge.polygon_to_avstack(poly_ros, header=None)
    assert isinstance(poly_2, Polygon)

    # check consistency
    assert np.allclose(poly.boundary, poly_2.boundary)
