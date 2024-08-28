"""Use these classes when you've already made the base dataset
as a rosbag and just want to do postprocessing for other reasons"""

import os
from typing import List

from avstack.config import HOOKS
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header
from tqdm import tqdm
from vision_msgs.msg import Detection3DArray

from avstack_bridge import (
    Bridge,
    DetectionBridge,
    GeometryBridge,
    ObjectStateBridge,
    TrackBridge,
    do_transform_detection3d,
)
from avstack_rosbag import rosbag_api
from avstack_rosbag.config import ROSBAG
from avstack_rosbag.utils import ConsoleLogger


@ROSBAG.register_module()
class DatasetPostprocessor:
    def __init__(
        self,
        bag_folder: str,
        bag_name: str,
        other_hooks: List[dict] = [],
    ):
        # set up the connection to the rosbag
        bag_filepath = os.path.join(bag_folder, bag_name, f"{bag_name}_0.db3")
        self.conn, self.c = rosbag_api.connect(bag_filepath)
        topic_names = rosbag_api.getAllTopicsNames(self.c, print_out=False)
        topic_types = rosbag_api.getAllMsgsTypes(self.c, print_out=False)
        self.type_map = {
            topic_names[i]: topic_types[i] for i in range(len(topic_types))
        }

        # initialize the hooks to run
        self.hooks = {}
        self.hooks["other"] = [
            HOOKS.build(hook) if isinstance(hook, dict) else hook
            for hook in other_hooks
        ]

        # load all the data
        self._idx = 0
        self._load_everything()

    def __len__(self):
        return len(self._get_messages_by_time(topic_name="/agent_names")["data"])

    def __iter__(self):
        return self

    def __next__(self):
        if self._idx < len(self._timestamps):
            return self.run_next()
        else:
            raise StopIteration

    def _get_messages_by_time(self, topic_name: str):
        # get the messages and message type
        t, msgs = rosbag_api.getAllMessagesInTopic(self.c, topic_name, print_out=False)
        try:
            msg_type = get_message(self.type_map[topic_name])
        except KeyError as e:
            print(f"From database {rosbag_api.get_database_names(self.c)}")
            raise e

        # loop and form the dictionary
        msgs_by_time = {"type": msg_type, "data": {}}
        for msg in msgs:
            x = deserialize_message(msg, msg_type)
            try:
                time = Bridge.rostime_to_time(x.header.stamp)
            except AttributeError:
                time = Bridge.rostime_to_time(x.transforms[0].header.stamp)
            if time not in msgs_by_time["data"]:
                msgs_by_time["data"][time] = x
            else:
                # assume only static transforms can be multi-time broadcast
                msgs_by_time["data"][time].transforms.extend(x.transforms)

        return msgs_by_time

    def _get_static_transform_messages(self, topic_name: str = "/tf_static"):
        """Get static tf messages in a dictionary by name"""
        msgs_tf_static = self._get_messages_by_time(topic_name=topic_name)
        tfs = {
            "{}-to-{}".format(tf.header.frame_id, tf.child_frame_id): tf
            for tfs_time in msgs_tf_static["data"].values()
            for tf in tfs_time.transforms
        }
        msgs_tf_static["data"] = tfs
        return msgs_tf_static

    def _get_dynamic_transform_messages(self, topic_name: str = "/tf"):
        msgs_tf = self._get_messages_by_time(topic_name=topic_name)
        tfs = {}
        for timestamp, transforms in msgs_tf["data"].items():
            tfs[timestamp] = {
                "{}-to-{}".format(tf.header.frame_id, tf.child_frame_id): tf
                for tf in transforms.transforms
            }
        msgs_tf["data"] = tfs
        return msgs_tf

    def _load_everything(self):
        self._data = {
            "msgs_tf": self._get_dynamic_transform_messages(topic_name="/tf"),
            "msgs_tf_static": self._get_static_transform_messages(
                topic_name="/tf_static"
            ),
            "msgs_agents_active": self._get_messages_by_time(topic_name="/agent_names"),
            "msgs_tracks_fused": self._get_messages_by_time("/command_center/tracks"),
            "msgs_objs": self._get_messages_by_time("/object_truth"),
        }
        agent_names = list(self._data["msgs_agents_active"]["data"].values())[0].agents
        self._data["agent_state"] = {
            agent: self._get_messages_by_time("/{}/state".format(agent))
            for agent in agent_names
        }
        self._data["msgs_fov"] = {
            agent: self._get_messages_by_time("/{}/lidar0/fov".format(agent))
            for agent in agent_names
        }
        self._data["msgs_det"] = {
            agent: self._get_messages_by_time("/{}/lidar0/detections".format(agent))
            for agent in agent_names
        }
        self._data["msgs_tracks_agents"] = {
            agent: self._get_messages_by_time("/{}/lidar0/tracks".format(agent))
            for agent in agent_names
        }
        self._timestamps = list(self._data["msgs_tf"]["data"].keys())

    def run_next(self, logger=ConsoleLogger):
        # get next timestamp
        timestamp = self._timestamps[self._idx]
        all_objs_truth = None

        # run this iteration
        agents_now = self._data["msgs_agents_active"]["data"][timestamp]
        if timestamp in self._data["msgs_fov"][agents_now.agents[0]]["data"]:
            # get transform information
            tf_now = self._data["msgs_tf"]["data"][timestamp]
            global_header = Header(
                frame_id="world", stamp=Bridge.time_to_rostime(timestamp)
            )

            # run conversions for global things
            all_objs_truth = ObjectStateBridge.objectstatearray_to_avstack(
                self._data["msgs_objs"]["data"][timestamp]
            )
            fused_trks_global = TrackBridge.tracks_to_avstack(
                self._data["msgs_tracks_fused"]["data"][timestamp]
            )

            # preallocate data structures
            all_agents = {}
            all_fovs_global = {}
            all_dets_global = {}
            all_trks_global = {}

            # run conversions for each agent
            for agent in agents_now.agents:
                # get the full transformation
                w_to_a = "{}-to-{}".format("world", agent)
                a_to_s = "{}-to-{}".format(agent, "{}/lidar0".format(agent))
                tf_sensor_to_agent = self._data["msgs_tf_static"]["data"][a_to_s]
                try:
                    tf_agent_to_world = self._data["msgs_tf_static"]["data"][w_to_a]
                except KeyError:
                    tf_agent_to_world = tf_now[w_to_a]

                # store agent states
                all_agents[agent] = ObjectStateBridge.objectstate_to_avstack(
                    msg_obj=self._data["agent_state"][agent]["data"][timestamp]
                )

                # perform frame transformations in ROS
                dets_sensor_ros = self._data["msgs_det"][agent]["data"][timestamp]
                dets_agent_ros = [
                    do_transform_detection3d(det_sensor, tf_sensor_to_agent)
                    for det_sensor in dets_sensor_ros.detections
                ]
                dets_global_ros = [
                    do_transform_detection3d(det_agent, tf_agent_to_world)
                    for det_agent in dets_agent_ros
                ]
                dets_global_ros = Detection3DArray(
                    header=global_header, detections=dets_global_ros
                )

                # convert with rosbridge
                all_dets_global[agent] = DetectionBridge.detectionarray_to_avstack(
                    dets_global_ros
                )
                all_fovs_global[agent] = GeometryBridge.polygon_to_avstack(
                    self._data["msgs_fov"][agent]["data"][timestamp]
                )
                all_trks_global[agent] = TrackBridge.tracks_to_avstack(
                    self._data["msgs_tracks_agents"][agent]["data"][timestamp]
                )

            # other hooks that get everything as inputs but can't affect things
            for hook in self.hooks["other"]:
                hook(
                    agents=all_agents,
                    field_of_view_agents=all_fovs_global,
                    detections_agents=all_dets_global,
                    tracks_agents=all_trks_global,
                    tracks_fused=fused_trks_global,
                    truths=all_objs_truth,
                    logger=logger,
                )

        # increment for next time
        self._idx += 1

        return all_objs_truth


if __name__ == "__main__":
    bag_folder = "/data/shared/CARLA/rosbags/"
    bag_name = "baseline_algorithms"
    postproc = DatasetPostprocessor(bag_folder=bag_folder, bag_name=bag_name)

    print(f"Loaded postprocessor of length {len(postproc)}")

    for _ in tqdm(range(30)):
        postproc.run_next()
