import numpy as np
from avstack.calibration import (
    Calibration,
    CameraCalibration,
    LidarCalibration,
    RadarCalibration,
)
from avstack.environment.objects import VehicleState
from avstack.geometry import (
    Acceleration,
    AngularVelocity,
    Attitude,
    Box3D,
    GlobalOrigin3D,
    PassiveReferenceFrame,
    PointMatrix3D,
    Position,
    Velocity,
    q_stan_to_cam,
)
from avstack.modules.tracking import BasicBoxTrack3D
from avstack.sensors import LidarData


# -- calibration data

# ref_lidar = ReferenceFrame(
#     x=np.array([0, 0, 1.73]), q=np.quaternion(1), reference=GlobalOrigin3D
# )
# ref_camera = ReferenceFrame(
#     x=np.array([0.27, 0.06, 1.65]), q=q_stan_to_cam, reference=GlobalOrigin3D
# )
ref_world = PassiveReferenceFrame(frame_id="world", timestamp=0.0)
ref_agent = PassiveReferenceFrame(frame_id="agent", timestamp=0.0)
ref_camera = PassiveReferenceFrame(frame_id="camera", timestamp=0.0)
ref_lidar = PassiveReferenceFrame(frame_id="lidar", timestamp=0.0)

P_cam = np.array(
    [
        [7.215377000000e02, 0.000000000000e00, 6.095593000000e02, 4.485728000000e01],
        [0.000000000000e00, 7.21537000000e02, 1.728540000000e02, 2.163791000000e-01],
        [0.000000000000e00, 0.000000000000e00, 1.000000000000e00, 2.745884000000e-03],
    ]
)

img_shape = (375, 1242, 3)
camera_calib = CameraCalibration(ref_camera, P_cam, img_shape)
box_calib = Calibration(ref_agent)
lidar_calib = LidarCalibration(ref_lidar)
radar_calib = RadarCalibration(ref_agent, fov_horizontal=np.pi, fov_vertical=np.pi / 2)


def get_point_cloud(seed: int = None, reference=GlobalOrigin3D) -> LidarData:
    if seed is not None:
        np.random.seed(seed)
    frame = 0
    timestamp = 0.0
    calibration = LidarCalibration(reference=reference)
    data = PointMatrix3D(x=np.random.randn(1000, 4), calibration=calibration)
    pc_data = LidarData(
        frame=frame,
        timestamp=timestamp,
        source_ID="lidar",
        calibration=calibration,
        data=data,
    )
    return pc_data


def get_object_global(seed, reference=GlobalOrigin3D) -> VehicleState:
    np.random.seed(seed)
    pos_obj = Position(10 * np.random.rand(3), reference)
    rot_obj = Attitude(q_stan_to_cam, reference)
    box_obj = Box3D(pos_obj, rot_obj, [2, 2, 5])  # box in local coordinates
    vel_obj = Velocity(10 * np.random.rand(3), reference)
    acc_obj = Acceleration(np.random.rand(3), reference)
    ang_obj = AngularVelocity(np.quaternion(1), reference)
    obj = VehicleState("car")
    obj.set(0, pos_obj, box_obj, vel_obj, acc_obj, rot_obj, ang_obj)
    return obj


def get_box_3d(seed, reference=GlobalOrigin3D) -> Box3D:
    np.random.seed(seed)
    pos_obj = Position(10 * np.random.rand(3), reference)
    rot_obj = Attitude(q_stan_to_cam, reference)
    box_obj = Box3D(pos_obj, rot_obj, [2, 2, 5])  # box in local coordinates
    return box_obj


def get_boxtrack_3d(seed, ID_track=10, reference=GlobalOrigin3D) -> BasicBoxTrack3D:
    np.random.seed(seed)
    box_track_1 = BasicBoxTrack3D(
        t0=0.0,
        box3d=get_box_3d(seed=seed, reference=reference),
        ID_force=ID_track,
        obj_type="car",
        reference=reference,
    )
    return box_track_1
