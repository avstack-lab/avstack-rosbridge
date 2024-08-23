from avapi.carla.dataset import CarlaScenesManager
from avstack.geometry import GlobalOrigin3D, PassiveReferenceFrame
from avstack.modules.perception.fov_estimator import ConcaveHullLidarFOVEstimator
from avstack.modules.perception.object3d import MMDetObjectDetector3D
from avstack.modules.tracking.multisensor import MeasurementBasedMultiTracker
from avstack.modules.tracking.tracker3d import BasicBoxTracker3D
from std_msgs.msg import Header, String

from avstack_bridge.base import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.geometry import GeometryBridge
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.sensors import CameraSensorBridge, LidarSensorBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import AgentArray, ObjectStateArray


class CarlaDatasetLoader:
    def __init__(
        self,
        dataset_path: str,
        scene_idx: int,
        i_frame_start: int = 0,
        run_perception: bool = False,
        run_tracking: bool = False,
        run_fusion: bool = False,
        perception_hook=None,
        tracking_hook=None,
    ) -> None:
        self.scene_dataset = CarlaScenesManager(
            data_dir=dataset_path
        ).get_scene_dataset_by_index(scene_idx=scene_idx)
        self.i_frame = i_frame_start - 1  # start after initialization
        self.agent_names = []

        # set up perception algorithm
        self.run_perception = run_perception
        if self.run_perception:
            self.perception = {
                "lidar": {
                    "vehicle": MMDetObjectDetector3D(
                        model="pointpillars",
                        dataset="carla-vehicle",
                    ),
                    "infrastructure": MMDetObjectDetector3D(
                        model="pointpillars",
                        dataset="carla-infrastructure",
                    ),
                },
                "fov": ConcaveHullLidarFOVEstimator(max_height=5),
            }
        else:
            self.perception = {}
        self.perception_hook = perception_hook

        # set up tracking algorithms
        self.run_tracking = run_tracking
        if self.run_tracking:
            self.tracking = {
                "detections_3d": {
                    "base": BasicBoxTracker3D,
                    "agents": {},
                }
            }
        self.tracking_hook = tracking_hook

        # set up fusion algorithms
        self.run_fusion = run_fusion
        if self.run_fusion:
            self.fusion = MeasurementBasedMultiTracker(tracker=BasicBoxTracker3D())

    def load_next(self, logger) -> ObjectStateArray:
        """Loads the next set of data from the dataset

        'next' is defined in this case as the next available frame of NPC data
        """
        # timestamp and frame info
        self.i_frame += 1
        objs = None
        if self.i_frame >= len(self.scene_dataset.frames):
            raise SystemExit
        frame = self.scene_dataset.frames[self.i_frame]
        timestamp = self.scene_dataset.get_timestamp(frame=frame)
        global_header = Header(
            frame_id="world", stamp=Bridge.time_to_rostime(timestamp)
        )

        # Process objects once loaded
        objs = self.scene_dataset.get_objects_global(frame=self.i_frame)
        # convert object messages
        obj_header = Bridge.reference_to_header(
            PassiveReferenceFrame(
                frame_id="world",
                timestamp=timestamp,
            )
        )
        objs_msgs = ObjectStateBridge.avstack_to_objecstatearray(
            obj_states=objs, header=obj_header
        )
        # HACK: ensure that timestamp is correct
        objs_msgs.header.stamp = Bridge.time_to_rostime(timestamp)

        all_fovs = {}
        all_dets_global = {}
        sensor_poses = {}
        agent_types = {}
        agent_poses = {}
        agent_data = {}
        agent_percep = {}
        agent_tracks = {}
        agent_objects = {}
        agents = self.scene_dataset.get_agents(frame=frame)
        self.agent_names = [f"agent{agent.ID}" for agent in agents]
        agent_names_msg = AgentArray(
            header=obj_header,
            agents=[String(data=agent) for agent in self.agent_names],
        )
        for agent in agents:
            if agent.ID is None:
                raise RuntimeError("Agent must have an ID")
            agent_name = f"agent{agent.ID}"

            # agent poses
            agent_ref = agent.as_reference()
            agent_ref.from_frame = "world"
            agent_ref.to_frame = agent_name
            agent_ref.timestamp = timestamp
            agent_poses[agent_name] = Bridge.reference_to_tf2_stamped(agent_ref)
            agent_types[agent_name] = agent.obj_type

            # agent sensor data
            data = {}
            percep = {}
            tracks = {}
            for sensor in self.scene_dataset.sensor_IDs[agent.ID]:
                if frame in self.scene_dataset.get_frames(
                    sensor=sensor, agent=agent.ID
                ):
                    sensor_name = sensor.replace("-", "")
                    sensor_frame = f"{agent_name}/{sensor_name}"
                    sensor_ref = PassiveReferenceFrame(
                        frame_id=sensor_frame,
                        timestamp=timestamp,
                    )
                    sensor_header = Bridge.reference_to_header(sensor_ref)
                    if "camera" in sensor:
                        raw_data = self.scene_dataset.get_image(
                            frame=frame,
                            sensor=sensor,
                            agent=agent.ID,
                        )
                        raw_dets = None  # TODO: add camera perceptions
                        fov_est = None
                        sensor_data = CameraSensorBridge.avstack_to_imgmsg(
                            raw_data, header=sensor_header
                        )
                    elif "lidar" in sensor:
                        raw_data = self.scene_dataset.get_lidar(
                            frame=frame,
                            sensor=sensor,
                            agent=agent.ID,
                        )
                        if self.run_perception:
                            model = (
                                "infrastructure"
                                if "static" in agent.obj_type
                                else "vehicle"
                            )
                            raw_dets = self.perception["lidar"][model](raw_data)
                            dets_name = "detections_3d"
                            trks_name = "tracks_3d"
                            raw_data_global = raw_data.project(GlobalOrigin3D)
                            fov_est = self.perception["fov"](raw_data_global)
                            all_fovs[agent_name] = fov_est
                            percep["fov"] = GeometryBridge.avstack_to_polygon(
                                fov_est, stamped=True, header=global_header
                            )
                        else:
                            raw_dets = None
                        sensor_data = LidarSensorBridge.avstack_to_pc2(
                            raw_data, header=sensor_header
                        )
                    else:
                        sensor_data = None
                        raw_dets = None
                        fov_est = None

                    # store sensor data
                    if sensor_data is not None:
                        data[sensor_name] = sensor_data
                        raw_data.reference.from_frame = agent_name
                        raw_data.reference.to_frame = sensor_frame
                        raw_data.reference.timestamp = timestamp
                        sensor_poses[sensor_frame] = Bridge.reference_to_tf2_stamped(
                            raw_data.reference
                        )

                    # handle perception outputs
                    if raw_dets is not None:
                        # pass to the perception hook
                        if self.perception_hook is not None:
                            raw_dets = self.perception_hook(
                                detections=raw_dets,
                                reference=raw_data.reference,
                                agent_name=agent_name,
                                sensor_name=sensor_name,
                                logger=logger,
                            )

                        # store perception data
                        percep[dets_name] = DetectionBridge.avstack_to_detectionarray(
                            raw_dets,
                            header=sensor_header,
                        )

                        # pass perception data to tracker
                        if self.run_tracking:
                            # initialize tracker if needed
                            if agent.ID not in self.tracking[dets_name]["agents"]:
                                self.tracking[dets_name]["agents"][
                                    agent.ID
                                ] = self.tracking[dets_name]["base"](
                                    check_reference=False
                                )
                            # perform trackingin global reference frame
                            raw_dets_global = raw_dets.apply_and_return(
                                "change_reference", GlobalOrigin3D, inplace=False
                            )
                            all_dets_global[agent_name] = raw_dets_global
                            # pass detections to tracker
                            raw_tracks_global = self.tracking[dets_name]["agents"][
                                agent.ID
                            ](
                                raw_dets_global,
                                platform=GlobalOrigin3D,
                                check_reference=False,
                            )
                            # pass to the tracking hook
                            if self.tracking_hook is not None:
                                raw_tracks_global = self.tracking_hook(
                                    tracks=raw_tracks_global,
                                    reference=raw_data.reference,
                                    agent_name=agent_name,
                                    sensor_name=sensor_name,
                                    logger=logger,
                                )
                            # convert track type
                            tracks[trks_name] = TrackBridge.avstack_to_tracks(
                                tracks=raw_tracks_global,
                                header=global_header,
                            )

            # save data for the agent
            agent_data[agent_name] = data
            agent_percep[agent_name] = percep
            agent_tracks[agent_name] = tracks

            # object states in agent view
            try:
                agent_objs = self.scene_dataset.get_objects(
                    frame=frame,
                    sensor="lidar-0",
                    agent=agent.ID,
                    max_dist=60,
                )
            except FileNotFoundError:
                pass
            else:
                passive_ref = PassiveReferenceFrame(
                    frame_id=agent_name, timestamp=timestamp
                )
                agent_obj_header = Bridge.reference_to_header(passive_ref)
                agent_objects[
                    agent_name
                ] = ObjectStateBridge.avstack_to_objecstatearray(
                    agent_objs,
                    header=agent_obj_header,
                )

        # run fusion
        if self.run_fusion:
            raw_fused_tracks = self.fusion(
                detections=all_dets_global, fovs=all_fovs, check_reference=False
            )
            fused_tracks = TrackBridge.avstack_to_tracks(
                tracks=raw_fused_tracks,
                header=global_header,
            )
        else:
            fused_tracks = None

        return (
            agent_names_msg,
            objs_msgs,
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
        )
