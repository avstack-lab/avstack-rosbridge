from avapi.carla import CarlaScenesManager

from avstack_rosbag import DatasetLoader


def test_carla_dataset_loader():
    # initialize the dataset loader
    CSM = CarlaScenesManager(data_dir="/data/shared/CARLA/multi-agent-v1")
    dataset_loader = DatasetLoader(
        scene_manager=CSM,
        scene_idx=0,
        i_frame_start=4,
    )

    # load a few frames of data
    loaded_sensor_data = False
    for _ in range(10):
        data_out = dataset_loader.load_next()
        if "lidar0" in data_out["agents_sensors_data"]["agent0"]:
            loaded_sensor_data = True

    assert loaded_sensor_data  # at least one frame should get data
