# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from functools import partial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time

import rosbag2_py

from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2 as LidarMsg

from avstack_msgs.msg import AgentArray, ObjectStateArray


class CarlaDatasetRecorder(Node):

    def __init__(self):
        super().__init__('carla_dataset_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='carla_dataset_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # we don't know a-prior the number of agents
        self.initialized_agents = False

        # ========================
        # topics
        # ========================
        
        self._qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        # TOPIC: active agents
        self.create_rosbag_and_subscription(
            topic="active_agents",
            msg_name="avstack_msgs/msg/AgentArray",
            msg_type=AgentArray,
        )

        # TOPIC: object ground truth, global
        self.create_rosbag_and_subscription(
            topic="object_truth",
            msg_name="avstack_msgs/msg/ObjectStateArray",
            msg_type=ObjectStateArray,
        )

        # TOPICS FOR AGENTS
        self.sub_agent_pose = {}
        self.sub_agent_obj_truth = {}
        self.sub_agent_sensor_data = {}

    def create_rosbag_and_subscription(self, topic: str, msg_name: str, msg_type):
        topic_info = rosbag2_py._storage.TopicMetadata(
            name=topic,
            type=msg_name,
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)
        self.sub_obj_truth = self.create_subscription(
            msg_type,
            topic,
            partial(self.topic_callback, topic),
            qos_profile=self._qos,
        )

    def initialize_agent_topics(self, n_agents: int):
        for i_agent in range(n_agents):
            # TOPIC: object ground truth, agent
            self.create_rosbag_and_subscription(
                topic=f"agent{i_agent}/object_truth",
                msg_name="avstack_msgs/msg/ObjectStateArray",
                msg_type=ObjectStateArray,
            )
        
            # TOPIC: agent sensor data
            # HACK: only consider primaries for now
            sensor_msgs = [
                ("camera0", "sensor_msgs/msg/Image", ImageMsg),
                ("lidar0", "sensor_msgs/msg/PointCloud2", LidarMsg)
            ]
            for sensor, msg_name, msg_type in sensor_msgs:
                self.create_rosbag_and_subscription(
                    topic=f"agent{i_agent}/{sensor}",
                    msg_name=msg_name,
                    msg_type=msg_type,
                )
        self.initialized_agents = True

    def topic_callback(self, topic: str, msg):
        """Generic writer"""
        if topic == "active_agents":
            self.initialize_agent_topics(n_agents=len(msg.agents))
        elif not self.initialized_agents:
            raise RuntimeError(
                f"We must set the agents before writing to topics, instead got {topic}"
            )
        self.writer.write(
            topic,
            serialize_message(msg),
            Time.from_msg(msg.header.stamp).nanoseconds,
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        sbr = CarlaDatasetRecorder()
        rclpy.spin(sbr)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        rclpy.shutdown()


if __name__ == '__main__':
    main()