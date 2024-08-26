import os
import shutil

from avapi.carla import CarlaScenesManager

from avstack_rosbag import (
    DatasetAlgorithms,
    DatasetLoader,
    DatasetRosConverter,
    DatasetRosWriter,
)


def run_writer(loader, algortihms, converter):
    # init/run the writer
    bag_name = "test_bag"
    try:
        dataset_roswriter = DatasetRosWriter(
            loader=loader,
            algorithms=algortihms,
            rosconverter=converter,
            bag_name=bag_name,
        )

        # loop a few frames
        for _ in range(10):
            dataset_roswriter.run_next()
        assert os.path.exists(bag_name)
    finally:
        if os.path.exists(bag_name):
            shutil.rmtree(bag_name)


def test_roswriter_no_algorithms():
    # initialize the dataset loader
    CSM = CarlaScenesManager(data_dir="/data/shared/CARLA/multi-agent-v1")
    dataset_loader = DatasetLoader(
        scene_manager=CSM,
        scene_idx=0,
        i_frame_start=4,
    )

    # inintialize the ros converter/writer
    dataset_rosconverter = DatasetRosConverter()

    # run it all
    run_writer(dataset_loader, None, dataset_rosconverter)


def test_roswriter_with_algorithms():
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

    # inintialize the ros converter/writer
    dataset_rosconverter = DatasetRosConverter()

    # run it all
    run_writer(dataset_loader, dataset_algorithms, dataset_rosconverter)
