import numpy as np
from avstack.geometry import PointMatrix3D
from avstack.sensors import ImageData, LidarData
from sensor_msgs.msg import Image, PointCloud2
from utilities import camera_calib, lidar_calib

from avstack_bridge.sensors import CameraSensorBridge, LidarSensorBridge


t = 0.0
frame = 0
lidar_ID = 0
camera_ID = 1

arr = np.random.randint(0, 255, size=camera_calib.img_shape, dtype="uint8")
img_data = ImageData(t, frame, arr, camera_calib, camera_ID)

x = np.random.randn(10000, 4)
pm = PointMatrix3D(x, lidar_calib)
pc_data = LidarData(t, frame, pm, lidar_calib, lidar_ID)


def test_avstack_to_imgmsg():
    cs_bridge = CameraSensorBridge()
    imgmsg = cs_bridge.avstack_to_imgmsg(img_data)
    assert isinstance(imgmsg, Image)


def test_imgmsg_to_avstack():
    cs_bridge = CameraSensorBridge()
    imgmsg = cs_bridge.avstack_to_imgmsg(img_data)
    img_data2 = cs_bridge.imgmsg_to_avstack(imgmsg, calibration=camera_calib)
    assert isinstance(img_data2, ImageData)


def test_avstack_to_pc2():
    li_bridge = LidarSensorBridge()
    pc2msg = li_bridge.avstack_to_pc2(pc_data)
    assert isinstance(pc2msg, PointCloud2)


def test_pc2_to_avstack():
    li_bridge = LidarSensorBridge()
    pc2msg = li_bridge.avstack_to_pc2(pc_data)
    pc_data2 = li_bridge.pc2_to_avstack(pc2msg, calibration=lidar_calib)
    assert isinstance(pc_data2, LidarData)
