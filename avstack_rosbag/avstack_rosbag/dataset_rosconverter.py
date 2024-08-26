from typing import Dict

from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage

from avstack_bridge.base import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.geometry import GeometryBridge
from avstack_bridge.metrics import MetricsBridge
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.sensors import CameraSensorBridge, LidarSensorBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import AgentArray
from avstack_rosbag.config import ROSBAG


@ROSBAG.register_module()
class DatasetRosConverter:
    def __init__(self):
        self._static_tfs_published = set()

    def __call__(
        self,
        data_out: Dict,
        alg_out: Dict,
    ) -> Dict:
        """Convert data and output from algorithms to ROS messages"""

        #########################################################
        # PREALLOCATE DATA STRUCTURES
        #########################################################

        objs_msgs = None
        agent_name_msgs = None
        tfs = []
        tfs_static = []
        agent_sensor_obj_msgs = {}
        agent_sensor_data_msgs = {}
        agent_percep_msgs = {}
        agent_track_msgs = {}
        agent_percep_metrics_msgs = {}
        agent_fov_metrics_msgs = {}
        agent_track_metrics_msgs = {}
        fusion_msgs = None
        fusion_metrics_msgs = None

        #########################################################
        # CENTARLIZED THINGS
        #########################################################

        timestamp = data_out["timestamp"]
        now_stamp = Bridge.time_to_rostime(timestamp)
        global_header = Header(
            frame_id="world",
            stamp=now_stamp,
        )

        # object truths
        objs_msgs = {
            "type": "avstack_msgs/msg/ObjectStateArray",
            "data": ObjectStateBridge.avstack_to_objecstatearray(
                obj_states=data_out["objs_global_truth"],
                header=global_header,
            ),
        }

        # agent names
        agent_name_msgs = {
            "type": "avstack_msgs/msg/AgentArray",
            "data": AgentArray(
                header=global_header,
                agents=[String(data=agent_name) for agent_name in data_out["agents"]],
            ),
        }

        # centralized fusion (**in global**)
        if "fusion" in alg_out:
            fusion_msgs = {
                "type": "avstack_msgs/msg/BoxTrackArray",
                "data": TrackBridge.avstack_to_tracks(
                    tracks=alg_out["fusion"],
                    header=global_header,
                ),
            }

        # metrics on centarlized fusion
        if "metrics" in alg_out:
            if "fusion" in alg_out["metrics"]:
                fusion_metrics_msgs = {
                    "type": "avstack_msgs/msg/AssignmentMetrics",
                    "data": MetricsBridge.assignment_metrics_avstack_to_ros(
                        metrics=alg_out["metrics"]["fusion"],
                    ),
                }

        #########################################################
        # LOOPING OVER AGENTS
        #########################################################

        # loop over agents
        for agent_name, agent in data_out["agents"].items():
            # -- agent reference frame
            ref_name = f"world-to-{agent_name}"
            if "static" in agent.obj_type:
                if ref_name not in self._static_tfs_published:
                    agent_ref = agent.as_reference()
                    agent_ref.from_frame = "world"
                    agent_ref.to_frame = agent_name
                    agent_ref.timestamp = 0.0  # time doesn't matter for static
                    tfs_static.append(Bridge.reference_to_tf2_stamped(agent_ref))
                    self._static_tfs_published.add(ref_name)
            else:
                agent_ref = agent.as_reference()
                agent_ref.from_frame = "world"
                agent_ref.to_frame = agent_name
                agent_ref.timestamp = timestamp
                tfs.append(Bridge.reference_to_tf2_stamped(agent_ref))

            # -- map frame is first agent pose
            ref_map = "world-to-map"
            if ref_map not in self._static_tfs_published:
                map_ref = agent.as_reference()
                map_ref.from_frame = "world"
                map_ref.to_frame = "map"
                map_ref.timestamp = 0.0  # time doesn't matter for static
                tfs_static.append(Bridge.reference_to_tf2_stamped(map_ref))
                self._static_tfs_published.add(ref_map)

            # -- sensor things
            sensor_obj_msgs = {}
            sensor_data_msgs = {}
            sensor_percep_msgs = {}
            sensor_fov_msgs = {}
            sensor_track_msgs = {}
            sensor_percep_metrics_msgs = {}
            sensor_fov_metrics_msgs = {}
            sensor_track_metrics_msgs = {}
            for sensor_name, sensor_ref in data_out["agents_sensors"][
                agent_name
            ].items():

                # -- sensor reference frame: assume static relative to agent
                ref_name = f"{agent_name}-to-{sensor_name}"
                if ref_name not in self._static_tfs_published:
                    if sensor_ref is not None:
                        sensor_ref.from_frame = agent_name
                        sensor_ref.to_frame = f"{agent_name}/{sensor_name}"
                        sensor_ref.timestamp = 0.0  # time doesn't matter for static
                        tfs_static.append(Bridge.reference_to_tf2_stamped(sensor_ref))
                        self._static_tfs_published.add(ref_name)

                # -- header
                sensor_frame = f"{agent_name}/{sensor_name}"
                sensor_header = Header(
                    frame_id=sensor_frame,
                    stamp=now_stamp,
                )

                # -- sensor objects
                if (
                    data_out["agents_sensors_objects"][agent_name][sensor_name]
                    is not None
                ):
                    sensor_obj_msgs[sensor_name] = {
                        "type": "avstack_msgs/msg/ObjectStateArray",
                        "data": ObjectStateBridge.avstack_to_objecstatearray(
                            obj_states=data_out["agents_sensors_objects"][agent_name][
                                sensor_name
                            ],
                            header=sensor_header,
                        ),
                    }

                # -- sensor data
                sensor_data = data_out["agents_sensors_data"][agent_name][sensor_name]
                if sensor_data is not None:
                    if "camera" in sensor_name:
                        sensor_data_type = "sensor_msgs/msg/Image"
                        bridge_convert = CameraSensorBridge.avstack_to_imgmsg
                    elif "lidar" in sensor_name:
                        sensor_data_type = "sensor_msgs/msg/PointCloud2"
                        bridge_convert = LidarSensorBridge.avstack_to_pc2
                    else:
                        raise NotImplementedError(sensor_name)
                    sensor_data_msgs[sensor_name] = {
                        "type": sensor_data_type,
                        "data": bridge_convert(
                            sensor_data,
                            header=sensor_header,
                        ),
                    }

                # -- perception outputs (**in sensor frame**)
                if "perception" in alg_out:
                    # HACK: not all sensors have perception data yet
                    if (
                        "detections_local"
                        in alg_out["perception"][agent_name][sensor_name]
                    ):
                        # detections
                        dets_local = alg_out["perception"][agent_name][sensor_name][
                            "detections_local"
                        ]
                        if dets_local is not None:
                            sensor_percep_msgs[sensor_name] = {
                                "type": "vision_msgs/msg/Detection3DArray",
                                "data": DetectionBridge.avstack_to_detectionarray(
                                    dets=dets_local,
                                    header=sensor_header,
                                ),
                            }

                        # detection metrics
                        metrics_perception = alg_out["metrics"]["perception"][
                            agent_name
                        ][sensor_name]
                        if metrics_perception is not None:
                            sensor_percep_metrics_msgs[sensor_name] = {
                                "type": "avstack_msgs/msg/AssignmentMetrics",
                                "data": MetricsBridge.assignment_metrics_avstack_to_ros(
                                    metrics=metrics_perception,
                                ),
                            }

                    # HACK: not all sensors have perception data yet
                    if "fov_global" in alg_out["perception"][agent_name][sensor_name]:
                        # field of view
                        fov_global = alg_out["perception"][agent_name][sensor_name][
                            "fov_global"
                        ]
                        if fov_global is not None:
                            sensor_fov_msgs[sensor_name] = {
                                "type": "geometry_msgs/msg/PolygonStamped",
                                "data": GeometryBridge.avstack_to_polygon(
                                    polygon=fov_global,
                                    stamped=True,
                                    header=global_header,
                                ),
                            }

                        # field of view metrics
                        # metrics_fov = alg_out["metrics"]
                        metrics_fov = None
                        if metrics_fov is not None:
                            sensor_fov_metrics_msgs[sensor_name] = {
                                "type": "avstack_msgs/msg/FovMetrics",
                                "data": MetricsBridge.fov_metrics_avstack_to_ros(
                                    metrics=metrics_fov,
                                ),
                            }

                # -- tracking outputs (**in global frame**)
                if "tracking" in alg_out:
                    # HACK: not all sensors have perception data yet
                    if "tracks_global" in alg_out["tracking"][agent_name][sensor_name]:
                        # tracks
                        tracks_global = alg_out["tracking"][agent_name][sensor_name][
                            "tracks_global"
                        ]
                        if tracks_global is not None:
                            sensor_track_msgs[sensor_name] = {
                                "type": "avstack_msgs/msg/BoxTrackArray",
                                "data": TrackBridge.avstack_to_tracks(
                                    tracks=tracks_global,
                                    header=global_header,
                                ),
                            }

                        # track metrics
                        metrics_tracking = alg_out["metrics"]["tracking"][agent_name][
                            sensor_name
                        ]
                        if metrics_tracking is not None:
                            sensor_track_metrics_msgs[sensor_name] = {
                                "type": "avstack_msgs/msg/AssignmentMetrics",
                                "data": MetricsBridge.assignment_metrics_avstack_to_ros(
                                    metrics=metrics_tracking,
                                ),
                            }

            # -- store outputs
            agent_sensor_obj_msgs[agent_name] = sensor_obj_msgs
            agent_sensor_data_msgs[agent_name] = sensor_data_msgs
            agent_percep_msgs[agent_name] = sensor_percep_msgs
            agent_track_msgs[agent_name] = sensor_track_msgs
            agent_percep_metrics_msgs[agent_name] = sensor_percep_metrics_msgs
            agent_fov_metrics_msgs[agent_name] = sensor_fov_metrics_msgs
            agent_track_metrics_msgs[agent_name] = sensor_track_metrics_msgs

        # package up output - keys are base topics, using {} as placeholders
        ros_out = {
            "/object_truth": objs_msgs,
            "/agent_names": agent_name_msgs,
            "/{}/{}/object_truth": agent_sensor_obj_msgs,
            "/{}/{}/data": agent_sensor_data_msgs,
            "/{}/{}/detections": agent_percep_msgs,
            "/{}/{}/tracks": agent_track_msgs,
            "/command_center/tracks": fusion_msgs,
            "/metrics/{}/{}/detections": agent_percep_metrics_msgs,
            "/metrics/{}/{}/tracks": agent_track_metrics_msgs,
            "/metrics/fusion": fusion_metrics_msgs,
        }

        # convert current transforms to transform arrays
        if len(tfs_static) > 0:
            tf_static_msgs = {
                "type": "tf2_msgs/msg/TFMessage",
                "data": TFMessage(transforms=tfs_static),
            }
            ros_out["/tf_static"] = tf_static_msgs
        if len(tfs) > 0:
            tf_msgs = {
                "type": "tf2_msgs/msg/TFMessage",
                "data": TFMessage(transforms=tfs),
            }
            ros_out["/tf"] = tf_msgs

        # add additional topics from the hooks
        if "hooks" in alg_out:
            for hooks in alg_out["hooks"].values():
                for hook in hooks:
                    ros_out.update(**hook.ros_topic_write)

        return ros_out
