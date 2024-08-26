_base_ = ["baseline_algorithms.py"]


rosbag_writer = {
    "bag_name": "baseline_trust",
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
        "other_hooks": [
            {
                "type": "TrustFusionRosbagHook",
                "hook": {
                    "type": "TrustFusionHook",
                    "n_calls_burnin": 2,
                    "model": {
                        "type": "TrustEstimator",
                        "measurement": {
                            "type": "ViewBasedPsm",
                            "assign_radius": 1.0,
                        },
                        "updater": {
                            "type": "TrustUpdater",
                        },
                    },
                },
            },
        ],
    },
}
