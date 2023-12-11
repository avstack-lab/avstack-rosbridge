from avstack.environment.objects import ObjectState as ObjectStateAV
from avstack.geometry import PassiveReferenceFrame
from utilities import get_object_global

from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectState, ObjectStateArray


t0 = 0.0
passive_agent_reference = PassiveReferenceFrame(frame_id="agent", timestamp=t0)
passive_world_reference = PassiveReferenceFrame(frame_id="world", timestamp=t0)


def test_avstack_to_objectstate_world():
    obj_bridge = ObjectStateBridge()
    obj_state = get_object_global(seed=1, reference=passive_world_reference)
    obj_state_ros = obj_bridge.avstack_to_objectstate(
        obj_state=obj_state,
    )
    assert isinstance(obj_state_ros, ObjectState)


def test_objectstate_to_avstack_world():
    obj_bridge = ObjectStateBridge()
    obj_state = get_object_global(seed=1, reference=passive_world_reference)
    obj_state_ros = obj_bridge.avstack_to_objectstate(
        obj_state=obj_state,
    )
    obj_state_2 = obj_bridge.objectstate_to_avstack(
        msg_obj=obj_state_ros,
    )
    assert isinstance(obj_state_2, ObjectStateAV)


def test_avstack_to_objectstatearray_world():
    obj_bridge = ObjectStateBridge()
    obj_states = [
        get_object_global(seed=1, reference=passive_world_reference),
        get_object_global(seed=2, reference=passive_world_reference),
    ]
    obj_states_ros = obj_bridge.avstack_to_objecstatearray(
        obj_states=obj_states,
    )
    assert isinstance(obj_states_ros, ObjectStateArray)


def test_objectstate_to_avstack_world():
    obj_bridge = ObjectStateBridge()
    obj_states = [
        get_object_global(seed=1, reference=passive_world_reference),
        get_object_global(seed=2, reference=passive_world_reference),
    ]
    obj_states_ros = obj_bridge.avstack_to_objecstatearray(
        obj_states=obj_states,
    )
    obj_states_2 = obj_bridge.objectstatearray_to_avstack(
        msg_objarray=obj_states_ros,
    )
    for obj_state in obj_states_2:
        assert isinstance(obj_state, ObjectStateAV)
