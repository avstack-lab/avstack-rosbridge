# class CameraSensorBridge:
#     def __init__(self) -> None:
#         self.cv_bridge = CvBridge()
#         self.calib_bridge = CameraCalibrationBridge()

#     @staticmethod
#     def imgmsg_to_avstack(msg: Image) -> sensors.ImageData:
#         frame = None
#         ros_frame = msg.header.frame_id
#         timestamp = Bridge.rostime_to_time(msg.header.stamp)

#         img_data = sensors.ImageData(
#             frame=frame,
#             timestamp=timestamp,
#             source_ID=ros_frame,
#             calibration=calibration,
#             data=msg.data,
#         )

#         return img_data

#     @staticmethod
#     def avstack_to_imgmsg(img_data: sensors.ImageData) -> Image:
#         header = Bridge.reference_to_header(img_data.reference, img_data.timestamp)
#         imgmsg = self.cv_bridge.cv2_to_imgmsg(img_data.data)
#         imgmsg.header = header
#         return imgmsg
