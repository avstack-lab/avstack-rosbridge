from copy import deepcopy
from typing import Dict, List

from avstack.config import HOOKS
from avstack.geometry import GlobalOrigin3D
from avstack.metrics.assignment import get_instantaneous_metrics
from avstack.modules.perception.fov_estimator import ConcaveHullLidarFOVEstimator
from avstack.modules.perception.object3d import MMDetObjectDetector3D
from avstack.modules.tracking.multisensor import MeasurementBasedMultiTracker
from avstack.modules.tracking.tracker3d import BasicBoxTracker3D

from avstack_rosbag.config import ROSBAG


@ROSBAG.register_module()
class DatasetAlgorithms:
    def __init__(
        self,
        run_perception: bool = False,
        run_tracking: bool = False,
        run_fusion: bool = False,
        perception_hooks: List[dict] = [],
        tracking_hooks: List[dict] = [],
        fusion_hooks: List[dict] = [],
        other_hooks: List[dict] = [],
    ):
        self.hooks = {}

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
                    "fov": ConcaveHullLidarFOVEstimator(max_height=5),
                },
            }
            self.hooks["perception"] = [
                HOOKS.build(hook) if isinstance(hook, dict) else hook
                for hook in perception_hooks
            ]
        else:
            self.perception = {}

        # set up tracking algorithms
        self.run_tracking = run_tracking
        if self.run_tracking:
            self.tracking = {}
            self._base_3d_tracker = BasicBoxTracker3D
            self._base_2d_tracker = None
            self._last_tracks = {}
            self.hooks["tracking"] = [
                HOOKS.build(hook) if isinstance(hook, dict) else hook
                for hook in tracking_hooks
            ]

        # set up fusion algorithms
        self.run_fusion = run_fusion
        if self.run_fusion:
            self.fusion = MeasurementBasedMultiTracker(tracker=BasicBoxTracker3D())
            self.hooks["fusion"] = [
                HOOKS.build(hook) if isinstance(hook, dict) else hook
                for hook in fusion_hooks
            ]

        # set up other hooks
        self.hooks["other"] = [
            HOOKS.build(hook) if isinstance(hook, dict) else hook
            for hook in other_hooks
        ]

    def __call__(self, data_out: Dict, logger=None) -> Dict:
        """Run the algorithms on top of the simulated dataset"""

        # output data structure
        a_s_dict = {
            a_name: {s_name: {} for s_name in data_out["agents_sensors_data"][a_name]}
            for a_name in data_out["agents"]
        }
        alg_out = {"metrics": {}}
        if self.run_perception:
            alg_out["perception"] = deepcopy(a_s_dict)
            alg_out["metrics"]["perception"] = deepcopy(a_s_dict)
            if self.run_tracking:
                alg_out["tracking"] = deepcopy(a_s_dict)
                alg_out["metrics"]["tracking"] = deepcopy(a_s_dict)
            if self.run_fusion:
                alg_out["fusion"] = None
                alg_out["metrics"]["fusion"] = None

        #######################################################
        # LOOPS ON AGENTS AGENTS AND SENSORS
        #######################################################

        timestamp = data_out["timestamp"]

        # run perception and tracking
        for agent_name, agent in data_out["agents"].items():
            for sensor_name, sensor_data in data_out["agents_sensors_data"][
                agent_name
            ].items():
                sensor_ref = data_out["agents_sensors"][agent_name][sensor_name]

                # HACK: only doing lidar detection for now
                if "lidar" not in sensor_name:
                    continue

                #######################################################
                # PERCEPTION
                #######################################################

                if self.run_perception:

                    # if there is sensor data
                    if sensor_data is not None:
                        # -- object detection in local frame
                        model_sensor = "lidar"
                        model_type = (
                            "infrastructure"
                            if "static" in agent.obj_type
                            else "vehicle"
                        )
                        dets_local = self.perception[model_sensor][model_type](
                            sensor_data
                        )

                        # -- fov estimation in global frame
                        fov_local = self.perception[model_sensor]["fov"](
                            sensor_data,
                            in_global=False,
                        )
                        fov_global = self.perception[model_sensor]["fov"](
                            sensor_data,
                            in_global=True,
                        )

                        # -- run through additional perception hooks
                        for hook in self.hooks["perception"]:
                            dets_local, fov_global = hook(
                                detections=dets_local,
                                field_of_view=fov_global,
                                reference=sensor_ref,
                                agent_name=agent_name,
                                sensor_name=sensor_name,
                                logger=logger,
                            )

                        # -- run metrics
                        metrics_perception = get_instantaneous_metrics(
                            tracks=dets_local,
                            truths=data_out["agents_sensors_objects"][agent_name][
                                sensor_name
                            ],
                            timestamp=data_out["timestamp"],
                        )

                        # -- do conversions
                        dets_global = dets_local.apply_and_return(
                            "change_reference", GlobalOrigin3D, inplace=False
                        )

                    else:
                        dets_local = None
                        dets_global = None
                        fov_local = None
                        fov_global = None
                        metrics_perception = None

                    # -- store data
                    alg_out["perception"][agent_name][sensor_name][
                        "detections_local"
                    ] = dets_local
                    alg_out["perception"][agent_name][sensor_name][
                        "detections_global"
                    ] = dets_global
                    alg_out["perception"][agent_name][sensor_name][
                        "fov_local"
                    ] = fov_local
                    alg_out["perception"][agent_name][sensor_name][
                        "fov_global"
                    ] = fov_global
                    alg_out["metrics"]["perception"][agent_name][
                        sensor_name
                    ] = metrics_perception

                #######################################################
                # TRACKING
                #######################################################

                if self.run_perception and self.run_tracking:
                    # -- initialize tracking if needed
                    if agent_name not in self.tracking:
                        self.tracking[agent_name] = {}
                    if sensor_name not in self.tracking[agent_name]:
                        if "lidar" in sensor_name:
                            base_tracker = self._base_3d_tracker
                        elif "camera" in sensor_name:
                            base_tracker = self._base_2d_tracker
                        else:
                            raise NotImplementedError
                        self.tracking[agent_name][sensor_name] = base_tracker(
                            check_reference=False
                        )

                    # -- if there is detection data
                    if dets_global is not None:
                        # -- run tracking in global frame
                        tracks_global = self.tracking[agent_name][sensor_name](
                            detections=dets_global,
                            platform=GlobalOrigin3D,
                            check_reference=False,
                        )
                    else:
                        # -- just propagate tracks to current time
                        tracks_global = self.tracking[agent_name][
                            sensor_name
                        ].predict_tracks(
                            timestamp=timestamp,
                            platform=GlobalOrigin3D,
                            check_reference=False,
                        )

                    # -- run through tracking hooks
                    for hook in self.hooks["tracking"]:
                        tracks_global = hook(
                            tracks=tracks_global,
                            field_of_view=fov_global,
                            reference=sensor_ref,
                            agent_name=agent_name,
                            sensor_name=sensor_name,
                            logger=logger,
                        )

                    # -- convert to local sensor frame
                    if sensor_ref is not None:
                        tracks_local = tracks_global.apply_and_return(
                            "change_reference", sensor_ref, inplace=False
                        )
                    else:
                        tracks_local = None

                    # -- run metrics -- need to check if none in case we just propagated w/o data
                    agent_objs = data_out["agents_sensors_objects"][agent_name][
                        sensor_name
                    ]
                    if agent_objs is not None:
                        metrics_tracking = get_instantaneous_metrics(
                            tracks=tracks_local,
                            truths=data_out["agents_sensors_objects"][agent_name][
                                sensor_name
                            ],
                            timestamp=data_out["timestamp"],
                        )
                    else:
                        metrics_tracking = None

                    # -- store data
                    alg_out["tracking"][agent_name][sensor_name][
                        "tracks_global"
                    ] = tracks_global
                    alg_out["tracking"][agent_name][sensor_name][
                        "tracks_local"
                    ] = tracks_local
                    alg_out["metrics"]["tracking"][agent_name][
                        sensor_name
                    ] = metrics_tracking

        #######################################################
        # CENTRALIZED THINGS
        #######################################################

        # run fusion on detections
        if self.run_perception and self.run_tracking and self.run_fusion:
            # HACK only doing fusion on lidar detections for now
            all_fovs_global = {}
            all_dets_global = {}
            all_trks_global = {}
            for agent_name in data_out["agents"]:
                for sensor_name in data_out["agents_sensors_data"][agent_name]:
                    if "lidar" in sensor_name:
                        all_dets_global[agent_name] = alg_out["perception"][agent_name][
                            sensor_name
                        ]["detections_global"]
                        all_fovs_global[agent_name] = alg_out["perception"][agent_name][
                            sensor_name
                        ]["fov_global"]
                        all_trks_global[agent_name] = alg_out["tracking"][agent_name][
                            sensor_name
                        ]["tracks_global"]

            # -- run fusion on detections in global frame
            if any([d is not None for d in all_dets_global]):
                fused_trks_global = self.fusion(
                    detections=all_dets_global,
                    fovs=all_fovs_global,
                    check_reference=False,
                )
            else:
                # -- just propagate tracks to current time
                fused_trks_global = self.fusion.predict_tracks(
                    timestamp=timestamp,
                    platform=GlobalOrigin3D,
                    check_reference=False,
                )

            # -- run through fusion hooks
            for hook in self.hooks["fusion"]:
                fused_trks_global = hook(
                    tracks=fused_trks_global,
                    agent_name=agent_name,
                    sensor_name="lidar",
                    logger=logger,
                )

            # -- run metrics
            metrics_fusion = get_instantaneous_metrics(
                tracks=fused_trks_global,
                truths=data_out["objs_global_truth"],
                timestamp=data_out["timestamp"],
            )

            # -- store data
            alg_out["fusion"] = fused_trks_global
            alg_out["metrics"]["fusion"] = metrics_fusion

        # other hooks that get everything as inputs but can't affect things
        for hook in self.hooks["other"]:
            hook(
                agents=data_out["agents"],
                field_of_view_agents=all_fovs_global,
                detections_agents=all_dets_global,
                tracks_agents=all_trks_global,
                tracks_fused=fused_trks_global,
                truths=data_out["objs_global_truth"],
                logger=logger,
            )

        # pass the hooks out
        alg_out["hooks"] = self.hooks

        return alg_out
