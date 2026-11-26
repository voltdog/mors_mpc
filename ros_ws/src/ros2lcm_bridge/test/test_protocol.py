import pytest
from mors_ros_msgs.msg import ModeCommand, ModeResult

from ros2lcm_bridge_runtime.lcm_types.mors_msgs import (
    robot_mode_cmd_msg as Cmd, robot_mode_result_msg as Result,
)
from ros2lcm_bridge_runtime.protocol import Protocol, finite_vector


@pytest.fixture
def harness():
    now, sent, events = [0.], [], []
    protocol = Protocol(lambda msg: sent.append(Cmd.decode(msg.encode())), events.append,
                        clock=lambda: now[0])
    protocol.on_state()
    return protocol, now, sent, events


def reply(protocol, command, result, reason=''):
    message = Result()
    message.command = command
    message.result = result
    message.reason = reason
    protocol.on_result(message)


def target_for(command):
    return Cmd.CHANGE_KINEMATICS if command == Cmd.START_ACTION else Cmd.STANDING


@pytest.mark.parametrize('params', [Cmd.KNEE_INSIDE, Cmd.KNEE_OUTSIDE, Cmd.KNEE_BACK, Cmd.KNEE_FRONT])
def test_action_send_once_and_complete(harness, params):
    p, now, sent, events = harness
    first = p.command(Cmd.START_ACTION, Cmd.CHANGE_KINEMATICS, params)
    now[0] = .3
    p.tick()
    assert [(msg.command, msg.target, msg.params) for msg in sent] == [
        (Cmd.START_ACTION, Cmd.CHANGE_KINEMATICS, params)]
    reply(p, Cmd.START_ACTION, Result.ACCEPTED)
    reply(p, Cmd.START_ACTION, Result.ACCEPTED)
    assert first.result().result == ModeResult.ACCEPTED
    assert Cmd.START_ACTION in p.pending
    assert p.command(Cmd.START_ACTION, Cmd.CHANGE_KINEMATICS, params).result().result == ModeResult.BUSY
    assert p.command(Cmd.SET_MODE, Cmd.STANDING).result().result == ModeResult.BUSY
    assert len(sent) == 1
    reply(p, Cmd.START_ACTION, Result.SUCCEEDED)
    assert not p.pending
    assert [event.result for event in events] == [
        ModeResult.ACCEPTED, ModeResult.BUSY, ModeResult.BUSY, ModeResult.SUCCEEDED]
    assert not p.command(Cmd.SET_MODE, Cmd.STANDING).done()


def test_ros_action_constants_match_wire():
    for name in ('START_ACTION', 'CHANGE_KINEMATICS', 'KNEE_INSIDE', 'KNEE_OUTSIDE',
                 'KNEE_BACK', 'KNEE_FRONT'):
        assert getattr(ModeCommand, name) == getattr(Cmd, name)
    assert ModeCommand().params == 0


@pytest.mark.parametrize(('target', 'params'), [
    (Cmd.GIVE_PAW, 0), (Cmd.WAVE_PAW, 1), (255, 0),
    (Cmd.CHANGE_KINEMATICS, -1), (Cmd.CHANGE_KINEMATICS, 4), (Cmd.CHANGE_KINEMATICS, 255),
])
def test_invalid_action_is_not_sent(harness, target, params):
    p, _, sent, _ = harness
    result = p.command(Cmd.START_ACTION, target, params).result()
    assert result.result == ModeResult.REJECTED and result.reason == 'INVALID_ACTION_OR_PARAMS'
    assert not sent and not p.pending


@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.ESTOP, Cmd.CLEAR_FAULT])
def test_non_action_params_are_ignored(harness, command):
    p, _, sent, _ = harness
    p.command(command, Cmd.STANDING, 255)
    assert sent[-1].params == 0
    assert sent[-1].target == (Cmd.STANDING if command == Cmd.SET_MODE else 0)


def test_send_once_and_complete(harness):
    p, now, sent, events = harness
    first = p.command(Cmd.SET_MODE, Cmd.STANDING)
    now[0] = .3
    p.tick()
    assert [(msg.command, msg.target, msg.params) for msg in sent] == [
        (Cmd.SET_MODE, Cmd.STANDING, 0)]
    reply(p, Cmd.SET_MODE, Result.ACCEPTED)
    reply(p, Cmd.SET_MODE, Result.ACCEPTED)
    assert first.result().result == ModeResult.ACCEPTED
    reply(p, Cmd.SET_MODE, Result.SUCCEEDED)
    assert [event.result for event in events] == [ModeResult.ACCEPTED, ModeResult.SUCCEEDED]


def test_only_one_regular_command_in_flight(harness):
    p, *_ = harness
    first = p.command(Cmd.SET_MODE, Cmd.STANDING)
    assert p.command(Cmd.SET_MODE, Cmd.LOCOMOTION).result().result == ModeResult.BUSY
    reply(p, Cmd.SET_MODE, Result.SUCCEEDED)
    assert first.result().result == ModeResult.SUCCEEDED
    assert not p.command(Cmd.SET_MODE, Cmd.LOCOMOTION).done()


@pytest.mark.parametrize('estop_first', [True, False])
@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.START_ACTION])
def test_estop_interrupts_operation_with_distinct_reply(harness, estop_first, command):
    p, _, sent, events = harness
    mode = p.command(command, target_for(command))
    reply(p, command, Result.ACCEPTED)
    stop = p.command(Cmd.ESTOP, 255)
    assert [(msg.command, msg.target, msg.params) for msg in sent][-1] == (Cmd.ESTOP, 0, 0)
    results = [(Cmd.ESTOP, Result.SUCCEEDED), (command, Result.FAILED)]
    if not estop_first:
        results.reverse()
    reply(p, *results[0])
    assert stop.done() == estop_first
    reply(p, *results[1])
    assert stop.result().result == ModeResult.SUCCEEDED
    assert mode.result().result == ModeResult.ACCEPTED
    assert {event.result for event in events[1:]} == {ModeResult.SUCCEEDED, ModeResult.FAILED}


@pytest.mark.parametrize(('wire', 'ros'), [
    (Result.REJECTED, ModeResult.REJECTED),
    (Result.BUSY, ModeResult.BUSY),
    (Result.TIMEOUT, ModeResult.TIMEOUT),
    (Result.FAILED, ModeResult.FAILED),
    (Result.CANCELED, ModeResult.CANCELED),
])
@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.START_ACTION])
def test_terminal_codes_are_translated(harness, wire, ros, command):
    p, _, _, events = harness
    first = p.command(command, target_for(command))
    reply(p, command, wire, 'detail')
    assert first.result().result == ros
    assert events[-1].reason == 'detail'


@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.START_ACTION])
def test_ack_timeout_blocks_uncorrelated_late_reply(harness, command):
    p, now, sent, events = harness
    first = p.command(command, target_for(command))
    now[0] = 2.01
    p.on_state()
    p.tick()
    assert first.result().reason == 'ACK_UNKNOWN'
    assert p.command(command, target_for(command)).result().result == ModeResult.UNKNOWN
    count = len(events)
    reply(p, command, Result.SUCCEEDED)
    assert len(events) == count
    assert len(sent) == 1
    assert p.command(Cmd.ESTOP).result().result == ModeResult.UNKNOWN
    assert len(sent) == 2


@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.START_ACTION])
def test_disconnect_does_not_replay_operation_on_status_return(harness, command):
    p, now, sent, events = harness
    p.command(command, target_for(command))
    reply(p, command, Result.ACCEPTED)
    now[0] = 1.01
    p.tick()
    assert events[-1].reason == 'FSM_UNAVAILABLE'
    p.on_state()
    p.tick()
    assert len(sent) == 1
    assert p.command(command, target_for(command)).result().result == ModeResult.UNKNOWN


def test_every_received_status_refreshes_connection(harness):
    p, now, _, _ = harness
    first = p.command(Cmd.SET_MODE, Cmd.STANDING)
    now[0] = 1.01
    p.on_state()
    p.tick()
    assert not first.done()
    now[0] = 2.02
    p.tick()
    assert first.result().reason == 'FSM_UNAVAILABLE'


@pytest.mark.parametrize('command', [Cmd.SET_MODE, Cmd.START_ACTION])
def test_result_timeout(harness, command):
    p, now, _, events = harness
    p.command(command, target_for(command))
    reply(p, command, Result.ACCEPTED)
    now[0] = 30.01
    p.on_state()
    p.tick()
    assert events[-1].result == ModeResult.UNKNOWN
    assert events[-1].reason == 'RESULT_UNKNOWN'


def test_validation(harness):
    p, _, sent, _ = harness
    for code in (Cmd.CANCEL_ACTION, Cmd.SET_ALGORITHM, Cmd.SET_EXTERNAL, 255):
        assert p.command(code, 255).result().reason == 'INVALID_COMMAND_OR_MODE'
    for target in (1, Cmd.EXTERNAL, 255):
        assert p.command(Cmd.SET_MODE, target).result().reason == 'INVALID_COMMAND_OR_MODE'
    assert not sent
    p.command(Cmd.CLEAR_FAULT, 255)
    assert (sent[-1].command, sent[-1].target, sent[-1].params) == (Cmd.CLEAR_FAULT, 0, 0)


def test_float32_stream_validation():
    assert finite_vector([0., 1., -3.4e38])
    for value in (float('nan'), float('inf'), -float('inf'), 1e300):
        assert not finite_vector([0., value])
