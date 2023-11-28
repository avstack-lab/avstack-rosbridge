from avstack.environment.objects import ObjectState
from utilities import get_object_global

from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateStamped


def test_avstack_to_objectstate_world():
    obj_bridge = ObjectStateBridge()
    obj_state = get_object_global(seed=1)
    obj_state_ros = obj_bridge.avstack_to_objectstate(
        obj_state=obj_state, tf_buffer=None
    )
    assert isinstance(obj_state_ros, ObjectStateStamped)


def test_objectstate_to_avstack_world():
    obj_bridge = ObjectStateBridge()
    obj_state = get_object_global(seed=1)
    obj_state_ros = obj_bridge.avstack_to_objectstate(
        obj_state=obj_state, tf_buffer=None
    )
    obj_state_2 = obj_bridge.objectstate_to_avstack(
        msg_obj=obj_state_ros, tf_buffer=None
    )
    assert isinstance(obj_state_2, ObjectState)
