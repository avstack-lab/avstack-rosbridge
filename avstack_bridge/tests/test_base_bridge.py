import numpy as np
import quaternion
from avstack.geometry import GlobalOrigin3D, PassiveReferenceFrame, ReferenceFrame
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

from avstack_bridge.base import Bridge


def test_time_to_rostime_to_time():
    av_bridge = Bridge()
    t_float = 139583.114
    t_ros = av_bridge.time_to_rostime(t_float)
    t_float2 = av_bridge.rostime_to_time(t_ros)
    assert isinstance(t_ros, Time)
    assert t_float == t_float2


def test_reference_to_tf2_stamped():
    base_bridge = Bridge()
    ref = ReferenceFrame(
        x=np.random.randn(3),
        q=np.quaternion(*np.random.rand(3)),
        reference=GlobalOrigin3D,
        timestamp=1.01,
        from_frame="world",
        to_frame="frame1",
    )
    tf2 = base_bridge.reference_to_tf2_stamped(ref)
    assert isinstance(tf2, TransformStamped)
    assert tf2.header.frame_id == "world"
    assert tf2.child_frame_id == "frame1"
    assert Bridge.rostime_to_time(tf2.header.stamp) == 1.01
    tf_x = np.array(
        [
            tf2.transform.translation.x,
            tf2.transform.translation.y,
            tf2.transform.translation.z,
        ]
    )
    tf_q = np.quaternion(
        tf2.transform.rotation.w,
        tf2.transform.rotation.x,
        tf2.transform.rotation.y,
        tf2.transform.rotation.z,
    )
    assert np.allclose(-ref.x, tf_x)
    assert quaternion.allclose(ref.q.conjugate(), tf_q)


def test_reference_to_header():
    base_bridge = Bridge()
    ref = PassiveReferenceFrame(frame_id="world", timestamp=1.01)
    header = base_bridge.reference_to_header(ref)
    assert isinstance(header, Header)
    assert header.frame_id == "world"
