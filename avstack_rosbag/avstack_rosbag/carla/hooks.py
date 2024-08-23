from typing import TYPE_CHECKING, Dict


if TYPE_CHECKING:
    from avsec.multi_agent.adversary import AdversaryModel
    from avstack.datastructs import DataContainer
    from avstack.geometry import ReferenceFrame


class AlgorithmHook:
    pass


class AdversaryHook:
    def __init__(self, models: Dict[tuple, "AdversaryModel"], verbose: bool = True):
        self.models = models
        self.verbose = verbose

    def __call__(
        self,
        detections: "DataContainer",
        reference: "ReferenceFrame",
        agent_name: str,
        sensor_name: str,
        logger=None,
    ):
        """Call the adversary hook on the detection data

        Args:
            detections: datacontainer of perception detections
            reference: reference frame for agent's sensor
            agent_name: agent's name
            sensor_name: sensor's name
        """
        if (agent_name, sensor_name) in self.models:
            n_before = len(detections)
            detections = self.models[(agent_name, sensor_name)](
                objects=detections, reference=reference
            )
            n_after = len(detections)
            if self.verbose:
                if logger is not None:
                    logger.info(f"Detections: {n_before} -> {n_after}")
        else:
            if self.verbose:
                if logger is not None:
                    logger.info(
                        f"({agent_name}, {sensor_name}) not in models ({self.models.keys()})"
                    )

        return detections
