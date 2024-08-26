from avapi.carla import CarlaScenesManager

from avstack_rosbag import DatasetAlgorithms, DatasetLoader


def test_algorithms_on_carla_dataset_no_hooks():
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

    # loop a few frames
    ran_tracking = False
    ran_fusion = False
    for _ in range(10):
        data_out = dataset_loader.load_next()
        alg_out = dataset_algorithms(data_out)
        if "tracks_global" in alg_out["tracking"]["agent0"]["lidar0"]:
            ran_tracking = True
        if alg_out["fusion"] is not None:
            ran_fusion = True

    # at least one frame should run tracking and fusion
    assert ran_tracking
    assert ran_fusion
