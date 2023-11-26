from avstack import sensors
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from .base import Bridge
from .calibration import CameraCalibrationBridge


class CameraSensorBridge(Bridge):
    def __init__(self) -> None:
        self.cv_bridge = CvBridge()
        self.calib_bridge = CameraCalibrationBridge()

    def imgmsg_to_avstack(self, msg: Image) -> sensors.ImageData:
        frame = None
        ros_frame = msg.header.frame_id
        calibration = self.calib_bridge(
            ctype="CameraCalibration",
        )
        timestamp = self.rostime_to_time(msg.header.stamp)

        img_data = sensors.ImageData(
            frame=frame,
            timestamp=timestamp,
            source_ID=ros_frame,
            calibration=calibration,
            data=msg.data,
        )

        return img_data

    def avstack_to_imgmsg(self, img_data: sensors.ImageData) -> Image:
        header = self.reference_to_header(img_data.reference, img_data.timestamp)
        imgmsg = self.cv_bridge.cv2_to_imgmsg(img_data.data)
        imgmsg.header = header
        return imgmsg
