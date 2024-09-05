from typing import TYPE_CHECKING, Dict, Union


if TYPE_CHECKING:
    from avapi._dataset import BaseSceneManager

from avstack.config import DATASETS
from avstack.geometry import GlobalOrigin3D

from avstack_rosbag.config import ROSBAG


@ROSBAG.register_module()
class DatasetLoader:
    def __init__(
        self,
        scene_manager: "BaseSceneManager",
        scene_idx: int,
        i_frame_start: int = 0,
        n_frames_trim: int = 0,
        n_frames_max: Union[int, None] = None,
    ) -> None:
        """TODO: make this a part of the scene dataset"""
        scene_manager = (
            DATASETS.build(scene_manager)
            if isinstance(scene_manager, dict)
            else scene_manager
        )
        self.scene_dataset = scene_manager.get_scene_dataset_by_index(
            scene_idx=scene_idx
        )
        self.i_frame = i_frame_start - 1  # start after initialization
        self.n_frames_max = len(self.scene_dataset.frames) - n_frames_trim
        if n_frames_max is not None:
            self.n_frames_max = min(self.n_frames_max, n_frames_max)

    def __iter__(self):
        return self

    def __next__(self):
        try:
            return self.load_next()
        except SystemExit:
            raise StopIteration

    def __len__(self):
        return self.n_frames_max

    def load_next(self) -> Dict:
        """Loads the next set of data from the dataset

        'next' is defined in this case as the next available frame of NPC data
        """

        # timestamp and frame info
        self.i_frame += 1
        objs_global_truth = None
        if self.i_frame >= self.n_frames_max:
            raise SystemExit
        frame = self.scene_dataset.frames[self.i_frame]
        timestamp = self.scene_dataset.get_timestamp(frame=frame)

        # Process  once loaded
        objs_global_truth = self.scene_dataset.get_objects_global(frame=self.i_frame)

        # avstack data structures
        agents = {}
        agents_sensors = {}
        agents_sensors_objects = {}
        agents_sensors_data = {}

        #######################################################
        # LOOPS ON AGENTS AGENTS AND SENSORS
        #######################################################

        # loops
        for agent in self.scene_dataset.get_agents(frame=frame):
            # get agent name
            if agent.ID is None:
                raise RuntimeError("Agent must have an ID")
            agent_name = f"agent{agent.ID}"
            agents[agent_name] = agent

            # preallocate data structures
            sensors_data = {}
            sensors_reference = {}
            sensors_objects = {}

            # loop over sensors
            for sensor_name in self.scene_dataset.sensor_IDs[agent.ID]:
                # HACK: handle the discrepancy in sensor naming
                sensor_name_ros = sensor_name.replace("-", "")
                # if the sensor has data this frame
                if frame in self.scene_dataset.get_frames(
                    sensor=sensor_name, agent=agent.ID
                ):
                    # load the sensor data
                    if "camera" in sensor_name:
                        sensors_data[sensor_name_ros] = self.scene_dataset.get_image(
                            frame=frame,
                            sensor=sensor_name,
                            agent=agent.ID,
                        )
                    elif "lidar" in sensor_name:
                        sensors_data[sensor_name_ros] = self.scene_dataset.get_lidar(
                            frame=frame,
                            sensor=sensor_name,
                            agent=agent.ID,
                        )
                    else:
                        continue  # TODO build in more sensors later
                    sensors_reference[sensor_name_ros] = sensors_data[
                        sensor_name_ros
                    ].reference

                    # object states in sensor view
                    try:
                        sensors_objects[
                            sensor_name_ros
                        ] = self.scene_dataset.get_objects(
                            frame=frame,
                            sensor=sensor_name,
                            agent=agent.ID,
                            max_dist=60,
                        )
                        # we need to keep these in the global frame, unfortunately
                        for obj in sensors_objects[sensor_name_ros]:
                            obj.change_reference(GlobalOrigin3D, inplace=True)
                    except FileNotFoundError:
                        pass
                else:
                    # set things as none if there's no data
                    sensors_data[sensor_name_ros] = None
                    sensors_objects[sensor_name_ros] = None

                    # since sensors are fixed, we can use the agent
                    # pose and the last known sensor fix to get the
                    # reference frame TODO
                    sensors_reference[sensor_name_ros] = None

            # save data for the agent
            agents_sensors[agent_name] = sensors_reference
            agents_sensors_data[agent_name] = sensors_data
            agents_sensors_objects[agent_name] = sensors_objects

        # package up for outputs
        data_out = {
            "frame": frame,
            "timestamp": timestamp,
            "objs_global_truth": objs_global_truth,
            "agents": agents,
            "agents_sensors": agents_sensors,
            "agents_sensors_objects": agents_sensors_objects,
            "agents_sensors_data": agents_sensors_data,
        }

        return data_out
