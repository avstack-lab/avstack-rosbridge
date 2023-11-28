import numpy as np
from avstack.sensors import ImageData
from sensor_msgs.msg import Image
from utilities import camera_calib

from avstack_bridge.sensors import CameraSensorBridge


t = 0.0
frame = 0
camera_ID = 1
arr = np.random.randint(0, 255, size=camera_calib.img_shape, dtype="uint8")
img_data = ImageData(t, frame, arr, camera_calib, camera_ID)


def test_avstack_to_imgmsg():
    cs_bridge = CameraSensorBridge()
    imgmsg = cs_bridge.avstack_to_imgmsg(img_data)
    assert isinstance(imgmsg, Image)
