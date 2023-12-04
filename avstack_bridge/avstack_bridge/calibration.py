import itertools

import numpy as np
from avstack.calibration import CameraCalibration
from sensor_msgs.msg import CameraInfo

from .base import Bridge


def flatten(alist):
    return list(itertools.chain.from_iterable(alist))


class CameraCalibrationBridge:
    @staticmethod
    def calib_to_caminfomsg(
        calib: CameraCalibration,
        header=None,
    ) -> CameraInfo:
        if not header:
            header = Bridge.reference_to_header(
                reference=calib.reference,
            )
        D_vec = []
        K_mat = flatten(calib.P[:3, :3].tolist())
        R_mat = flatten(np.eye(3).tolist())
        P_mat = flatten(calib.P.tolist())
        caminfo = CameraInfo(
            header=header,
            height=calib.height,
            width=calib.width,
            distortion_model="plumb_bob",
            d=D_vec,
            k=K_mat,
            r=R_mat,
            p=P_mat,
            binning_x=1,
            binning_y=1,
            # roi
        )
        return caminfo

    @staticmethod
    def caminfomsg_to_calib(caminfo: CameraInfo) -> CameraCalibration:
        reference = Bridge.header_to_reference(caminfo.header)
        P_mat = np.array(caminfo.P)
        height = caminfo.height
        width = caminfo.width
        calib = CameraCalibration(
            reference=reference,
            P=P_mat,
            img_shape=(height, width, 3),
        )
        return calib
