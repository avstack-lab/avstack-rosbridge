"""
A scenario demonstrating a false positive attack on agent 0's
lidar-based detections with markov propagation
"""

_base_ = ["../baseline_trust.py"]

rosbag_writer = {
    "bag_name": "cte_case_1",
    "algorithms": {
        "perception_hooks": [
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": True,
                    "models": {
                        ("agent0", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 0,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "fp_poisson": None,
                                "min_select": 3,
                                "max_select": 3,
                            },
                        },
                    },
                },
            }
        ],
    },
}
