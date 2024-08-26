import os
import shutil
from typing import TYPE_CHECKING, Union


if TYPE_CHECKING:
    from .dataset_algorithms import DatasetAlgorithms
    from .dataset_loader import DatasetLoader
    from .dataset_rosconverter import DatasetRosConverter

import rosbag2_py
from rclpy.serialization import serialize_message
from rclpy.time import Time

from avstack_rosbag.config import ROSBAG


class ConsoleLogger:
    @staticmethod
    def info(msg: str):
        print("\n" + msg)


@ROSBAG.register_module()
class DatasetRosWriter:
    def __init__(
        self,
        loader: Union[dict, "DatasetLoader"],
        algorithms: Union["DatasetAlgorithms", dict, None],
        rosconverter: Union["DatasetRosConverter", dict],
        bag_folder: str,
        bag_name: str,
        remove_bag_if_exists: bool = True,
    ):
        # dataset tools
        self.loader = ROSBAG.build(loader) if isinstance(loader, dict) else loader
        self.algorithms = (
            ROSBAG.build(algorithms) if isinstance(algorithms, dict) else algorithms
        )
        self.rosconverter = (
            ROSBAG.build(rosconverter)
            if isinstance(rosconverter, dict)
            else rosconverter
        )
        self._topics = set()

        # rosbag writer
        bag_filepath = os.path.join(bag_folder, bag_name)
        self.writer = rosbag2_py.SequentialWriter()
        if remove_bag_if_exists and os.path.exists(bag_filepath):
            shutil.rmtree(bag_filepath)
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_filepath, storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

    def __len__(self):
        return len(self.loader)

    def __iter__(self):
        return self

    def __next__(self):
        try:
            return self.run_next()
        except SystemExit:
            raise StopIteration

    @property
    def logger(self):
        return ConsoleLogger

    def _create_topic(
        self, topic_name: str, topic_type: str, serialization_format: str = "cdr"
    ):
        """
        Create a new topic.

        :param writer: writer instance
        :param topic_name:
        :param topic_type:
        :param serialization_format:
        :return:
        """
        topic_name = topic_name
        topic = rosbag2_py.TopicMetadata(
            name=topic_name, type=topic_type, serialization_format=serialization_format
        )
        self.writer.create_topic(topic)

    def _write(self, topic: str, msg, timestamp=None):
        if timestamp is None:
            try:
                timestamp_in_nanosecs = Time.from_msg(msg.header.stamp).nanoseconds
            except AttributeError:
                timestamp_in_nanosecs = Time.from_msg(
                    msg.transforms[0].header.stamp
                ).nanoseconds
        else:
            timestamp_in_nanosecs = int(1e9 * timestamp)

        self.writer.write(
            topic,
            serialize_message(msg),
            timestamp_in_nanosecs,
        )

    def _check_and_create_topic(self, topic_name: str, topic_type: str):
        if topic_name not in self._topics:
            self._create_topic(
                topic_name=topic_name,
                topic_type=topic_type,
            )
            self._topics.add(topic_name)

    def run_next(self):
        """Run the next frame of writing operations"""
        # get all the data etc
        data_out = self.loader.load_next()
        if self.algorithms is None:
            alg_out = {}
        else:
            alg_out = self.algorithms(data_out=data_out, logger=self.logger)
        ros_out = self.rosconverter(data_out=data_out, alg_out=alg_out)

        # write the topics -- keys are topic names with possible formatting options
        for topic_name, topic_data in ros_out.items():
            if topic_data is not None:
                # check how much formatting required
                n_formats = topic_name.count("{}")
                if n_formats == 0:
                    topic_format = topic_name
                    topic_type = topic_data["type"]
                    topic_msg = topic_data["data"]
                    self._check_and_create_topic(
                        topic_name=topic_format, topic_type=topic_type
                    )
                    self._write(topic=topic_format, msg=topic_msg)
                elif n_formats == 1:
                    # format via the agent name
                    for agent_name in topic_data:
                        topic_format = topic_name.format(agent_name)
                        topic_type = topic_data[agent_name]["type"]
                        topic_msg = topic_data[agent_name]["data"]
                        self._check_and_create_topic(
                            topic_name=topic_format, topic_type=topic_type
                        )
                        self._write(topic=topic_format, msg=topic_msg)
                elif n_formats == 2:
                    # format via the agent and sensor names
                    for agent_name in topic_data:
                        for sensor_name in topic_data[agent_name]:
                            topic_format = topic_name.format(agent_name, sensor_name)
                            topic_type = topic_data[agent_name][sensor_name]["type"]
                            topic_msg = topic_data[agent_name][sensor_name]["data"]
                            self._check_and_create_topic(
                                topic_name=topic_format, topic_type=topic_type
                            )
                            self._write(topic=topic_format, msg=topic_msg)
                else:
                    raise NotImplementedError(n_formats)

    def run_all(self):
        """Run all writing operations"""
        for _ in self:
            pass
