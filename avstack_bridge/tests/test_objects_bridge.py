from avstack.environment.objects import ObjectState as ObjectStateAV
from utilities import get_object_global

from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateArray, ObjectStateStamped


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
    assert isinstance(obj_state_2, ObjectStateAV)


def test_avstack_to_objectstatearray_world():
    obj_bridge = ObjectStateBridge()
    obj_states = [get_object_global(seed=1), get_object_global(seed=2)]
    obj_states_ros = obj_bridge.avstack_to_objecstatearray(
        obj_states=obj_states, tf_buffer=None
    )
    assert isinstance(obj_states_ros, ObjectStateArray)


def test_objectstate_to_avstack_world():
    obj_bridge = ObjectStateBridge()
    obj_states = [get_object_global(seed=1), get_object_global(seed=2)]
    obj_states_ros = obj_bridge.avstack_to_objecstatearray(
        obj_states=obj_states, tf_buffer=None
    )
    obj_states_2 = obj_bridge.objectstatearray_to_avstack(
        msg_objarray=obj_states_ros, tf_buffer=None
    )
    for obj_state in obj_states_2:
        assert isinstance(obj_state, ObjectStateAV)
