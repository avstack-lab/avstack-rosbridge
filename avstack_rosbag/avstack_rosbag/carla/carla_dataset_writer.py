import os

import rclpy
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from avstack_bridge import Bridge

from .carla_dataset_loader import CarlaDatasetLoader


class CarlaDatasetWriter(Node):
    def __init__(self):
        super().__init__("carla_dataset_writer")

        # dataset parameters
        self.declare_parameter(
            "output_folder", "/data/shared/CARLA/rosbags/carla_dataset"
        )
        self.declare_parameter("rosbag_name", "baseline")
        self.declare_parameter("real_time_framerate", 10.0)
        self.declare_parameter("dataset_path", "/data/shared/CARLA/multi-agent-v1")
        self.declare_parameter("scene_idx", 0)
        self.declare_parameter("i_frame_start", 4)
        self.declare_parameter("max_frames", 200)

        # prerun algorithm parameters
        self.declare_parameter("run_perception", False)
        self.declare_parameter("run_tracking", False)
        self.declare_parameter("run_fusion", False)
        self.declare_parameter("run_adversary", False)

        # log start info
        dp = self.get_parameter("dataset_path").value
        fm = self.get_parameter("max_frames").value
        fr = self.get_parameter("real_time_framerate").value
        self.get_logger().info(f"Replaying dataset {dp} for max {fm} frames at {fr} Hz")

        # hooks
        self.perception_hook = None
        self.tracking_hook = None
        self.loader = None

        # timer to run dataset writing
        timer_period = 1.0 / fr
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # rosbag writer
        self.writer = rosbag2_py.SequentialWriter()
        bag_path = os.path.join(
            self.get_parameter("output_folder").value,
            self.get_parameter("rosbag_name").value,
        )
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_path, storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        # =============================================
        # SET UP TOPICS
        # =============================================

        # TOPIC: metadata
        self.initialized = False
        self.create_topic(
            topic_name="initialization",
            topic_type="std_msgs/msg/String",
        )

        # TOPIC: transforms
        self.create_topic(
            topic_name="tf",
            topic_type="tf2_msgs/msg/TFMessage",
        )
        self.create_topic(
            topic_name="tf_static",
            topic_type="tf2_msgs/msg/TFMessage",
        )
        self.static_tfs_sent = set()

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
        self.topic_agent_percep = {}
        self.topic_agent_tracks = {}
        self.topic_fused_tracks = None

    def set_perception_hook(self, hook):
        self.perception_hook = hook

    def set_tracking_hook(self, hook):
        self.tracking_hook = hook

    def set_loader(self):
        # dataset replay options
        self.index = 0
        self.loader = CarlaDatasetLoader(
            dataset_path=self.get_parameter("dataset_path").value,
            scene_idx=self.get_parameter("scene_idx").value,
            i_frame_start=self.get_parameter("i_frame_start").value,
            run_perception=self.get_parameter("run_perception").value,
            run_tracking=self.get_parameter("run_tracking").value,
            run_fusion=self.get_parameter("run_fusion").value,
            perception_hook=self.perception_hook,
            tracking_hook=self.tracking_hook,
        )

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

    def write(self, topic, msg, timestamp=None):
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

    def send_transforms(
        self,
        agent_poses: dict,
        agent_types: dict,
        sensor_poses: dict,
        timestamp: float,
    ):

        #########################################################
        # STATIC TRANSFORMATIONS
        #########################################################
        tfs_static = []

        # map frame is the initial agent x-y position
        if "map" not in self.static_tfs_sent:
            tf_map = TransformStamped()
            tf_map.header.stamp = Bridge.time_to_rostime(timestamp)
            tf_map.header.frame_id = "world"
            tf_map.child_frame_id = "map"
            agent_0 = list(agent_poses.values())[0]
            tf_map.transform = agent_0.transform
            tf_map.transform.translation.z = 0.0
            tfs_static.append(tf_map)
            self.static_tfs_sent.add("map")

        # sensor poses
        for s_ID, s_pose in sensor_poses.items():
            if s_ID not in self.static_tfs_sent:
                tfs_static.append(s_pose)
                self.static_tfs_sent.add(s_ID)

        # static agent poses
        for a_ID, a_pose in agent_poses.items():
            if "static" in agent_types[a_ID]:
                if a_ID not in self.static_tfs_sent:
                    tfs_static.append(a_pose)
                    self.static_tfs_sent.add(a_ID)

        # only send static once
        if len(tfs_static) > 0:
            self.write("tf_static", TFMessage(transforms=tfs_static))

        #########################################################
        # DYNAMIC TRANSFORMATIONS
        #########################################################

        tfs_dynamic = [
            a_pose
            for a_ID, a_pose in agent_poses.items()
            if "static" not in agent_types[a_ID]
        ]
        self.write("tf", TFMessage(transforms=tfs_dynamic))

    def timer_callback(self):
        """Gather the data on a timer callback"""
        # set the loader if needed
        if self.loader is None:
            self.set_loader()

        # run the loader
        (
            agent_names,
            obj_state_array,
            agent_poses,
            agent_types,
            agent_data,
            agent_percep,
            agent_tracks,
            agent_objects,
            sensor_poses,
            fused_tracks,
            frame,
            timestamp,
        ) = self.loader.load_next(self.get_logger())

        # publish initial metadata
        if not self.initialized:
            init_msg = String(data="reset")
            self.write("initialization", init_msg, timestamp=timestamp)
            self.initialized = True

        # publish static transforms once
        self.send_transforms(
            agent_poses=agent_poses,
            agent_types=agent_types,
            sensor_poses=sensor_poses,
            timestamp=timestamp,
        )

        # publish the names of active agents
        self.write("active_agents", agent_names)

        # publish object ground truth object states
        self.write("object_truth", obj_state_array)

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

        # publish perception data
        for agent in agent_percep:
            if agent not in self.topic_agent_percep:
                self.topic_agent_percep[agent] = {}
            for det_name in agent_percep[agent]:
                # create publisher for detections
                if det_name not in self.topic_agent_percep[agent]:
                    if "detections_2d" in det_name:
                        msg_type = "vision_msgs/msg/Detection2DArray"
                    elif "detections_3d" in det_name:
                        msg_type = "vision_msgs/msg/Detection3DArray"
                    elif "fov" in det_name:
                        msg_type = "geometry_msgs/msg/PolygonStamped"
                    else:
                        raise ValueError(det_name)
                    self.create_topic(f"{agent}/{det_name}", msg_type)

                # only publish when data was captured
                if agent_percep[agent][det_name]:
                    self.write(f"{agent}/{det_name}", agent_percep[agent][det_name])

        # publish local tracking data
        for agent in agent_tracks:
            if agent not in self.topic_agent_tracks:
                self.topic_agent_tracks[agent] = {}
            for trk_name in agent_tracks[agent]:
                # create publisher for tracks
                if trk_name not in self.topic_agent_tracks[agent]:
                    if "tracks_2d" in det_name:
                        raise NotImplementedError(det_name)
                    elif "tracks_3d" in trk_name:
                        msg_type = "avstack_msgs/msg/BoxTrackArray"
                    else:
                        raise ValueError(trk_name)
                    self.create_topic(f"{agent}/{trk_name}", msg_type)

                # only publish when data was captured
                if agent_tracks[agent][trk_name]:
                    self.write(f"{agent}/{trk_name}", agent_tracks[agent][trk_name])

        # published fused track data
        if fused_tracks is not None:
            if self.topic_fused_tracks is None:
                self.create_topic(
                    f"command_center/tracks_3d", "avstack_msgs/msg/BoxTrackArray"
                )
                self.topic_fused_tracks = True
            self.write(f"command_center/tracks_3d", fused_tracks)

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
