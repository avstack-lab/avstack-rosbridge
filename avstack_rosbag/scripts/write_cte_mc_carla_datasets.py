from argparse import ArgumentParser

import avapi  # noqa # pylint: disable=unused-import
import avsec  # noqa # pylint: disable=unused-import
import avstack  # noqa # pylint: disable=unused-import
from avstack.config import Config
from tqdm import tqdm

from avstack_rosbag.config import ROSBAG


def main(args):
    # load base fp config
    cfg = Config.fromfile(args.config_filepath)

    #

    dataset_roswriter = ROSBAG.build(cfg["rosbag_writer"])

    # run all frames
    for _ in tqdm(dataset_roswriter):
        pass


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("config_filepath", type=str)
    args = parser.parse_args()
    main(args)
