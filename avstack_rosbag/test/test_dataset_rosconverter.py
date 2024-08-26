from avapi.carla import CarlaScenesManager

from avstack_rosbag import DatasetAlgorithms, DatasetLoader, DatasetRosConverter


def test_rosconverter_no_algorithms():
    # initialize the dataset loader
    CSM = CarlaScenesManager(data_dir="/data/shared/CARLA/multi-agent-v1")
    dataset_loader = DatasetLoader(
        scene_manager=CSM,
        scene_idx=0,
        i_frame_start=4,
    )

    # inintialize the ros converter
    dataset_rosconverter = DatasetRosConverter()

    # loop a few frames
    has_sensor_data = False
    for _ in range(10):
        data_out = dataset_loader.load_next()
        ros_out = dataset_rosconverter(data_out=data_out, alg_out={})
        if "agent0" in ros_out["/{}/{}/data"]["data"]:
            if "lidar0" in ros_out["/{}/{}/data"]["data"]["agent0"]:
                if ros_out["/{}/{}/data"]["data"]["agent0"]["lidar0"] is not None:
                    has_sensor_data = True

    assert has_sensor_data


def test_rosconverter_with_algorithms():
    # initialize the dataset loader
    CSM = CarlaScenesManager(data_dir="/data/shared/CARLA/multi-agent-v1")
    dataset_loader = DatasetLoader(
        scene_manager=CSM,
        scene_idx=0,
        i_frame_start=4,
    )

    # initialize the dataset algorithms
    dataset_algorithms = DatasetAlgorithms(
        run_perception=True,
        run_tracking=True,
        run_fusion=True,
        perception_hooks=[],
        tracking_hooks=[],
        fusion_hooks=[],
        other_hooks=[],
    )

    # inintialize the ros converter
    dataset_rosconverter = DatasetRosConverter()

    # loop a few frames
    has_sensor_data = False
    for _ in range(10):
        data_out = dataset_loader.load_next()
        alg_out = dataset_algorithms(data_out)
        ros_out = dataset_rosconverter(data_out=data_out, alg_out=alg_out)
