from avstack.config import HOOKS


class RosbagHook:
    def __init__(self, hook, verbose: bool = False):
        self.verbose = verbose
        self.ros_topic_write = {}
        self.hook = HOOKS.build(hook) if isinstance(hook, dict) else hook

    def __call__(self, *args, **kwargs):
        raise NotImplementedError
