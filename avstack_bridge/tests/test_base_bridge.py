from builtin_interfaces.msg import Time

from avstack_bridge.base import Bridge


# def test_ref_to_header():
#     raise


# def test_header_to_ref():
#     raise


# def test_tf2_from_header():
#     raise


def test_time_to_rostime_to_time():
    av_bridge = Bridge()
    t_float = 139583.114
    t_ros = av_bridge.time_to_rostime(t_float)
    t_float2 = av_bridge.rostime_to_time(t_ros)
    assert isinstance(t_ros, Time)
    assert t_float == t_float2
