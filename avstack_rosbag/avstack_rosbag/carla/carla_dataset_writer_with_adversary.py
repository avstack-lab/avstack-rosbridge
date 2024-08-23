import rclpy
from avsec.config import AVSEC
from avsec.multi_agent.adversary import AdversaryModel

from .carla_dataset_writer import CarlaDatasetWriter
from .hooks import AdversaryHook


def get_adversary_model(
    manifest: str,
    propagation: str,
    seed: int,
    dt_init: float = 1.0,
    dt_reset: float = 10.0,
) -> AdversaryModel:
    """Parse the inputs to the adversary model"""

    # parameters for manifest and propagation
    manifest_inputs = {
        "fp": {
            "type": "FalsePositiveManifest",
            "fp_poisson": 2.0,
            "x_sigma": 20.0,
            "hwl": [2, 2, 4],
        },
        "fn": {
            "type": "FalseNegativeManifest",
            "fn_poisson": 2.0,
        },
        "tr": {
            "type": "TranslationManifest",
            "tr_poisson": 2.0,
        },
    }
    propagation_inputs = {
        "static": {"type": "StaticPropagator"},
        "markov": {
            "type": "MarkovPropagator",
            "v_sigma": 10,
            "v_max": 12,
            "dv_sigma": 0.5,
        },
        "trajectory": {
            "type": "TrajectoryPropagator",
            "dr_total": 30,
            "dt_total": 5.0,
        },
    }

    # parse the manifest
    manifester = AVSEC.build(
        manifest_inputs[manifest],
        default_args={"seed": seed, "min_select": 1.0, "max_select": 10.0},
    )

    # parse the propagation
    if propagation.lower() != "none":
        propagator = AVSEC.build(
            propagation_inputs[propagation], default_args={"seed": seed}
        )
    else:
        propagator = None

    # build the model
    model = AdversaryModel(
        propagator=propagator,
        manifest=manifester,
        dt_init=dt_init,
        dt_reset=dt_reset,
        seed=seed,
    )
    return model


class CarlaDatasetWriterWithAdversary(CarlaDatasetWriter):
    def __init__(self):
        super().__init__()

        # adversary parameters
        self.declare_parameter("attack_seed", 0)
        self.declare_parameter("attacked_agents", [""])
        self.declare_parameter("attack_manifest", [""])
        self.declare_parameter("attack_propagation", [""])

        # repackage the adversary info data structure
        adv_parameters = {
            "agents": [
                v for v in self.get_parameter("attacked_agents").value if v != ""
            ],
            "manifest": [
                v for v in self.get_parameter("attack_manifest").value if v != ""
            ],
            "propagation": [
                v for v in self.get_parameter("attack_propagation").value if v != ""
            ],
        }

        # construct the perception hook
        adv_models = {
            (agent, "lidar0"): get_adversary_model(
                manifest=manifest,
                propagation=propagation,
                seed=self.get_parameter("attack_seed").value,
            )
            for agent, manifest, propagation in zip(*adv_parameters.values())
        }
        perception_hook = AdversaryHook(models=adv_models)

        # construct the tracking hook
        tracking_hook = None

        # initialize hooks
        super().set_perception_hook(perception_hook)
        super().set_tracking_hook(tracking_hook)


def main(args=None):
    rclpy.init(args=args)

    carla_writer = CarlaDatasetWriterWithAdversary()

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
