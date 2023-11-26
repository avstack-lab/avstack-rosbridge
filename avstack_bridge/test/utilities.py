import numpy as np
from avstack.calibration import (
    Calibration,
    CameraCalibration,
    LidarCalibration,
    RadarCalibration,
)
from avstack.geometry import GlobalOrigin3D, ReferenceFrame, q_stan_to_cam


# -- calibration data
ref_lidar = ReferenceFrame(
    x=np.array([0, 0, 1.73]), q=np.quaternion(1), reference=GlobalOrigin3D
)
ref_camera = ReferenceFrame(
    x=np.array([0.27, 0.06, 1.65]), q=q_stan_to_cam, reference=GlobalOrigin3D
)
P_cam = np.array(
    [
        [7.215377000000e02, 0.000000000000e00, 6.095593000000e02, 4.485728000000e01],
        [0.000000000000e00, 7.21537000000e02, 1.728540000000e02, 2.163791000000e-01],
        [0.000000000000e00, 0.000000000000e00, 1.000000000000e00, 2.745884000000e-03],
    ]
)

img_shape = (375, 1242, 3)
camera_calib = CameraCalibration(ref_camera, P_cam, img_shape)
box_calib = Calibration(ref_camera)
lidar_calib = LidarCalibration(ref_lidar)
radar_calib = RadarCalibration(ref_lidar, fov_horizontal=np.pi, fov_vertical=np.pi / 2)
