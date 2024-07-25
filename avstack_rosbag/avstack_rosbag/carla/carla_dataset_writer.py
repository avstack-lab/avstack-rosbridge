import os

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, TransformStamped

from .carla_dataset_loader import CarlaDatasetLoader


class CarlaDatasetWriter(Node):
    def __init__(self):
        super().__init__("carla_dataset_replayer")

        # parameters
        self.declare_parameter(
            "output_folder", "/data/shared/CARLA/rosbags/carla_dataset"
        )
        self.declare_parameter("real_time_framerate", 10.0)
        self.declare_parameter("dataset_path", "/data/shared/CARLA/multi-agent-v1")
        self.declare_parameter("scene_idx", 0)
        self.declare_parameter("i_frame_start", 4)
        self.declare_parameter("max_frames", 200)

        # log start info
        dp = self.get_parameter("dataset_path").value
        fm = self.get_parameter("max_frames").value
        fr = self.get_parameter("real_time_framerate").value
        self.get_logger().info(f"Replaying dataset {dp} for max {fm} frames at {fr} Hz")

        # dataset replay options
        self.index = 0
        timer_period = 1.0 / fr
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.loader = CarlaDatasetLoader(
            dataset_path=self.get_parameter("dataset_path").value,
            scene_idx=self.get_parameter("scene_idx").value,
            i_frame_start=self.get_parameter("i_frame_start").value,
        )

        # rosbag writer
        self.writer = rosbag2_py.SequentialWriter()
        bag_path = os.path.join(self.get_parameter("output_folder").value)
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_path, storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        # TOPIC: transforms
        self.create_topic(
            topic_name="tf",
            topic_type="tf2_msgs/msg/TFMessage",
        )

        # TOPIC: active agents
        self.create_topic(
            topic_name="active_agents",
            topic_type="avstack_msgs/msg/AgentArray",
        )

        # TOPIC: object ground truth, global
        self.create_topic(
            topic_name="object_truth",
            topic_type="avstack_msgs/msg/ObjectStateArray",
        )

        # TOPICS FOR AGENTS
        self.topic_agent_object_gt = {}
        self.topic_agent_data = {}

    def create_topic(
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

    def write(self, topic, msg):
        try:
            self.writer.write(
                topic,
                serialize_message(msg),
                Time.from_msg(msg.header.stamp).nanoseconds,
            )
        except AttributeError:  # assume list if doesn't have header at top level
            self.writer.write(
                topic,
                serialize_message(msg),
                Time.from_msg(msg.transforms[0].header.stamp).nanoseconds,
            )

    def timer_callback(self):
        (
            agent_names,
            obj_state_array,
            agent_poses,
            sensor_poses,
            agent_data,
            agent_objects,
            frame,
        ) = self.loader.load_next()

        # publish the names of active agents
        self.write("active_agents", agent_names)

        # publish object ground truth object states
        self.write("object_truth", obj_state_array)

        # publish agent and sensor pose information
        map_frame = Transf
        transforms = [map_frame] + list(agent_poses.values()) + list(sensor_poses.values())
        # self.get_logger().info(",".join([str(Bridge.rostime_to_time(tf.header.stamp)) for tf in transforms]))
        agent_poses_tf = TFMessage(transforms=transforms)
        self.write("tf", agent_poses_tf)

        # publish agent sensor data
        for agent in agent_data:
            if agent not in self.topic_agent_data:
                self.topic_agent_data[agent] = {}
            for sensor in agent_data[agent]:
                # create publisher if it doesn't exist
                if sensor not in self.topic_agent_data[agent]:
                    if "camera" in sensor:
                        msg_type = "sensor_msgs/msg/Image"
                    elif "lidar" in sensor:
                        msg_type = "sensor_msgs/msg/PointCloud2"
                    else:
                        raise ValueError(sensor)
                    self.create_topic(f"{agent}/{sensor}", msg_type)

                # only publish when data was captured
                if agent_data[agent][sensor]:
                    self.write(f"{agent}/{sensor}", agent_data[agent][sensor])

        # publish object information in agent view
        for agent in agent_objects:
            if agent not in self.topic_agent_object_gt:
                self.create_topic(
                    f"{agent}/object_truth", "avstack_msgs/msg/ObjectStateArray"
                )
            if agent_objects[agent] is not None:
                if agent_objects[agent]:
                    self.write(f"{agent}/object_truth", agent_objects[agent])

        # save index-to-frame map
        self.index += 1
        if (self.index % 20) == 0:
            self.get_logger().info(f"Completed {self.index} frames")
        if self.index >= self.get_parameter("max_frames").value:
            self.get_logger().info(f"Hit max frames...ending")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    carla_writer = CarlaDatasetWriter()

    try:
        rclpy.spin(carla_writer)
    except SystemExit:  # <--- process the exception
        rclpy.logging.get_logger("Quitting").info("Done")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    carla_writer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
