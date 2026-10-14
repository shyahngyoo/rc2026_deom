"""Lost transition replies must not lead to duplicate configuration."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

spec = importlib.util.spec_from_file_location(
    'activate_map', Path(__file__).resolve().parents[1] / 'launch' / 'activate_map.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state(value):
    response = GetState.Response()
    response.current_state.id = value
    response.current_state.label = str(value)
    return response


def check_sequence(responses, expected):
    activator = module.MapActivator(Mock(), timeout=1.0)
    activator.call = Mock(side_effect=responses)
    with patch.object(module.rclpy, 'ok', return_value=True):
        assert activator.run() == expected
    return [args.args[1].transition.id for args in activator.call.call_args_list
            if args.args[0] == 'change_state']


def test_lost_configure_reply_checks_state_before_activate():
    transitions = check_sequence([
        state(State.PRIMARY_STATE_UNCONFIGURED), None,
        state(State.PRIMARY_STATE_INACTIVE), ChangeState.Response(success=True),
        state(State.PRIMARY_STATE_ACTIVE),
    ], 0)
    assert transitions == [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_lost_activate_reply_and_get_state_reply_recover():
    assert check_sequence([
        None, state(State.PRIMARY_STATE_INACTIVE), None,
        state(State.PRIMARY_STATE_ACTIVE),
    ], 0) == [Transition.TRANSITION_ACTIVATE]


def test_real_configuration_rejection_stops_startup():
    assert check_sequence([
        state(State.PRIMARY_STATE_UNCONFIGURED), ChangeState.Response(success=False),
        state(State.PRIMARY_STATE_UNCONFIGURED),
    ], 1) == [Transition.TRANSITION_CONFIGURE]


def test_unreachable_server_has_bounded_timeout():
    activator = module.MapActivator(Mock(), timeout=0.005)
    activator.call = Mock(return_value=None)
    with patch.object(module.rclpy, 'ok', return_value=True):
        assert activator.run() == 1


def test_request_timeout_recreates_client_and_cleans_future():
    node = Mock()
    client = node.create_client.return_value
    future = client.call_async.return_value
    future.done.return_value = False
    activator = module.MapActivator(node, timeout=1.0, request_timeout=0.001)
    with patch.object(module.rclpy, 'ok', return_value=True), patch.object(module.rclpy, 'spin_once'):
        assert activator.call('get_state', GetState.Request()) is None
        assert activator.call('get_state', GetState.Request()) is None
    assert node.create_client.call_count == 2
    assert node.destroy_client.call_count == 2
    assert client.remove_pending_request.call_count == 2


def test_timeout_rebuilds_context_but_rejection_does_not():
    timed_out = Mock(retryable=True)
    timed_out.run.return_value = 1
    recovered = Mock(retryable=False)
    recovered.run.return_value = 0
    with patch.object(module.rclpy, 'init') as init, patch.object(module.rclpy, 'shutdown') as shutdown, patch.object(module, 'Node'), patch.object(module, 'MapActivator', side_effect=[timed_out, recovered]):
        assert module.activate_map(60) == 0
        assert init.call_count == shutdown.call_count == 2
    rejected = Mock(retryable=False)
    rejected.run.return_value = 1
    with patch.object(module.rclpy, 'init') as init, patch.object(module.rclpy, 'shutdown'), patch.object(module, 'Node'), patch.object(module, 'MapActivator', return_value=rejected):
        assert module.activate_map(60) == 1
        assert init.call_count == 1
