from types import SimpleNamespace

import pytest
from mors_msgs import robot_mode_cmd_msg as Cmd

from ros2lcm_bridge_runtime.protocol import Protocol, finite_vector


@pytest.fixture
def harness():
    now, sent, events = [0.], [], []
    protocol = Protocol(Cmd, lambda msg: sent.append(msg.encode()), events.append,
                        clock=lambda: now[0])
    state = lambda: SimpleNamespace(
        current_state=0, target_state=0, phase=0, algorithm_id=0, action_id=-1,
        fault=False, stub_mode=True)
    protocol.on_state(state())
    return protocol, now, sent, events, state


def reply(request, code, reason=''):
    return SimpleNamespace(command=request.wire.command, result=code, reason=reason)


def test_send_once_and_complete(harness):
    p, now, sent, events, state = harness
    request = p.command(0, 2)
    now[0] = .3
    p.tick()
    assert len(sent) == 1
    p.on_result(reply(request, 0))
    assert request.first.result().result == 0
    assert not request.final.done()
    p.on_result(reply(request, 1))
    assert request.final.result().result == 1
    assert [e.result for e in events] == [0, 1]


def test_only_one_regular_command_in_flight(harness):
    p, *_ = harness
    first = p.command(0, 2)
    assert p.command(0, 3).first.result().result == 3
    p.on_result(reply(first, 1))
    assert not p.command(0, 3).first.done()


@pytest.mark.parametrize('action_result', [1, 4, 7])
@pytest.mark.parametrize('interrupt_first', [True, False])
def test_cancel_reply_cannot_complete_action(harness, action_result, interrupt_first):
    p, *_ = harness
    action = p.start_action(0)
    p.on_result(reply(action, 0))
    cancel = p.cancel_action()
    results = [(cancel, 1 if action_result == 4 else 2), (action, action_result)]
    if not interrupt_first:
        results.reverse()
    request, code = results[0]
    p.on_result(reply(request, code))
    other = results[1][0]
    assert not other.final.done()
    p.on_result(reply(*results[1]))
    assert action.final.result().result == action_result
    assert cancel.final.done()


def test_estop_while_action_runs(harness):
    p, *_ = harness
    action = p.start_action(0)
    p.on_result(reply(action, 0))
    stop = p.command(Cmd.ESTOP)
    p.on_result(reply(stop, 0))
    p.on_result(reply(action, 7))
    assert action.final.result().result == 7
    assert not stop.final.done()
    p.on_result(reply(stop, 1))
    assert stop.final.result().result == 1


def test_estop_does_not_wait_for_action_ack(harness):
    p, now, sent, *_ = harness
    action = p.start_action(0)
    stop = p.command(Cmd.ESTOP)
    assert len(sent) == 2
    p.on_result(reply(stop, 0))
    p.on_result(reply(action, 2))
    p.on_result(reply(stop, 1))
    assert action.final.result().result == 2
    assert stop.final.result().result == 1


def test_ack_timeout_blocks_uncorrelated_late_reply(harness):
    p, now, sent, events, state = harness
    request = p.start_action(0)
    now[0] = 2.01
    p.on_state(state()); p.tick()
    assert request.final.result().reason == 'ACK_UNKNOWN'
    assert p.start_action(0).first.result().result == 255
    p.on_result(reply(request, 1))
    assert request.final.result().result == 255
    assert len(sent) == 1
    # ESTOP is still transmitted after uncertainty, without claiming confirmation.
    assert p.command(Cmd.ESTOP).first.result().result == 255
    assert len(sent) == 2
    p.on_state(state())
    assert p.start_action(0).first.result().result == 255
    restarted = Protocol(Cmd, lambda _: None, lambda _: None, clock=lambda: now[0])
    restarted.on_state(state())
    assert not restarted.start_action(0).first.done()


def test_disconnect_does_not_replay_action_on_status_return(harness):
    p, now, sent, events, state = harness
    request = p.start_action(0)
    p.on_result(reply(request, 0))
    now[0] = 1.01
    p.tick()
    assert request.final.result().reason == 'FSM_UNAVAILABLE'
    p.on_state(state())
    p.tick()
    assert len(sent) == 1
    assert p.start_action(0).first.result().result == 255


def test_every_received_status_refreshes_connection(harness):
    p, now, sent, events, state = harness
    request = p.command(0, 2)
    now[0] = 1.01
    assert p.on_state(state())
    p.tick()
    assert not request.final.done()
    now[0] = 2.02
    p.tick()
    assert request.final.result().reason == 'FSM_UNAVAILABLE'
    assert p.command(0, 2).first.result().result == 255


def test_result_timeout(harness):
    p, now, sent, events, state = harness
    request = p.start_action(0)
    p.on_result(reply(request, 0))
    now[0] = 30.01
    p.on_state(state()); p.tick()
    assert request.final.result().reason == 'RESULT_UNKNOWN'


def test_validation(harness):
    p, now, sent, events, state = harness
    assert p.cancel_action().first.result().result == 2
    assert p.start_action(-1).first.result().reason == 'INVALID_ACTION_ID'
    assert p.start_action(256).first.result().reason == 'INVALID_ACTION_ID'
    for code in (1, 2):
        assert p.command(code).first.result().reason == 'INVALID_COMMAND_OR_MODE'
    assert p.command(0, 1).first.result().reason == 'INVALID_COMMAND_OR_MODE'
    for target in (-1, 256):
        assert p.command(Cmd.SET_ALGORITHM, target).first.result().reason == 'INVALID_ALGORITHM_ID'
    for target in (0, 255):
        request = p.command(Cmd.SET_ALGORITHM, target)
        assert Cmd.decode(sent[-1]).target == target
        p.on_result(reply(request, 1))
    assert p.command(Cmd.SET_EXTERNAL, 255).wire.target == 0


def test_action_id_is_forwarded_without_name_mapping(harness):
    p, now, sent, *_ = harness
    request = p.start_action(255)
    assert Cmd.decode(sent[-1]).target == 255
    p.on_result(reply(request, 2, 'Unknown action'))
    assert request.final.result().reason == 'Unknown action'


def test_float32_stream_validation():
    assert finite_vector([0., 1., -3.4e38])
    for value in (float('nan'), float('inf'), -float('inf'), 1e300):
        assert not finite_vector([0., value])
