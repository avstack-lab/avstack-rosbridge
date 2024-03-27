import numpy as np
import ros2_numpy
from avstack import sensors
from avstack.geometry import PointMatrix3D
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2 as LidarMsg

from avstack_bridge.base import Bridge


cv_bridge = CvBridge()


class CameraSensorBridge:
    @staticmethod
    def imgmsg_to_avstack(msg: ImageMsg) -> sensors.ImageData:
        frame = None
        ros_frame = msg.header.frame_id
        timestamp = Bridge.rostime_to_time(msg.header.stamp)

        img_data = sensors.ImageData(
            frame=frame,
            timestamp=timestamp,
            source_ID=ros_frame,
            calibration=calibration,
            data=msg.data,
        )

        return img_data

    @staticmethod
    def avstack_to_imgmsg(img_data: sensors.ImageData, header=None) -> ImageMsg:
        if not header:
            header = Bridge.reference_to_header(img_data.reference)
        imgmsg = cv_bridge.cv2_to_imgmsg(img_data.data)
        imgmsg.header = header
        return imgmsg


class LidarSensorBridge:
    @staticmethod
    def pc2_to_avstack(msg: LidarMsg) -> sensors.LidarData:
        frame = None
        ros_frame = msg.header.frame_id
        timestamp = Bridge.rostime_to_time(msg.header.stamp)
        data = PointMatrix3D(ros2_numpy.numpify(msg), calibration)
        pc_data = sensors.LidarData(
            frame=frame,
            timestamp=timestamp,
            source_ID=ros_frame,
            calibration=calibration,
            data=data,
        )

        return pc_data

    @staticmethod
    def avstack_to_pc2(pc_data: sensors.LidarData, header=None) -> LidarMsg:
        if not header:
            header = Bridge.reference_to_header(pc_data.reference)
        data = np.zeros(
            pc_data.data.shape[0],
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        data["x"] = pc_data.data.x[:, 0]
        data["y"] = pc_data.data.x[:, 1]
        data["z"] = pc_data.data.x[:, 2]
        pcmsg = ros2_numpy.msgify(LidarMsg, data)
        pcmsg.header = header
        return pcmsg
