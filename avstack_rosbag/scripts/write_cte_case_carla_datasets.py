import os
from argparse import ArgumentParser

import avapi  # noqa # pylint: disable=unused-import
import avsec  # noqa # pylint: disable=unused-import
import avstack  # noqa # pylint: disable=unused-import
import avtrust  # noqa # pylint: disable=unused-import
from avstack.config import Config
from tqdm import tqdm

from avstack_rosbag.config import ROSBAG


def main():
    # configs for case studies
    config_files = [
        "case_1.py",
        "case_2.py",
        "case_3.py",
        "case_4.py",
    ]

    # loop over configs
    for config_file in config_files:
        # build from config
        config_filepath = os.path.join("dataset_configs/cte_cases", config_file)
        cfg = Config.fromfile(config_filepath)
        dataset_roswriter = ROSBAG.build(cfg["rosbag_writer"])

        # run all frames
        for _ in tqdm(dataset_roswriter):
            pass


if __name__ == "__main__":
    parser = ArgumentParser()
    args = parser.parse_args()
    main(args)
