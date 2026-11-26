"""Keyboard behavior and asynchronous command lifecycle without robot outputs."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from mors_ros_msgs.msg import ModeCommand, ModeResult, ModeState
from mors_ros_msgs.srv import Command
from rclpy.task import Future

from mors_fsm_keyboard_control import mors_fsm_keyboard_control as keyboard


@pytest.fixture
def control(monkeypatch):
    now = SimpleNamespace(value=10.0)
    monkeypatch.setattr(keyboard.time, 'monotonic', lambda: now.value)
    monkeypatch.setattr(keyboard.MorsFsmKeyboardControl, '_setup_keyboard_input', lambda self: None)
    monkeypatch.setattr(keyboard.MorsFsmKeyboardControl, '_write_plain', lambda self, text: None)
    rclpy.init()
    node = keyboard.MorsFsmKeyboardControl()
    node.timer.cancel()
    node.cmd_vel_pub = Mock()
    node.cmd_pose_pub = Mock()
    node.gait_params_pub = Mock()
    node.command_cli = Mock()
    node.command_cli.service_is_ready.return_value = True
    node.command_cli.call_async.side_effect = lambda request: Future()
    node._on_state(ModeState(current_state=ModeState.STANDING))
    try:
        yield node, now
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def complete(node, code=ModeResult.SUCCEEDED):
    future = node.command_future
    future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    node._on_result(ModeResult(result=code))


def enable_walking(node):
    node._handle_keyboard_key(' ')
    complete(node)


def test_enter_sends_one_native_command_and_waits_for_completion(control):
    node, _ = control
    node._on_state(ModeState(current_state=ModeState.PASSIVE))
    node._handle_keyboard_key('\n')
    node.timer_callback()
    requests = node.command_cli.call_async.call_args_list
    assert len(requests) == 1
    command = requests[0].args[0].command
    assert (command.command, command.target) == (ModeCommand.SET_MODE, ModeCommand.STANDING)
    assert node.fsm_state.current_state == ModeState.PASSIVE
    future = node.command_future
    future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    assert node.pending_target == ModeCommand.STANDING
    node._on_state(ModeState(current_state=ModeState.STANDING))
    node._on_result(ModeResult(result=ModeResult.SUCCEEDED))
    assert node.pending_target is None
    node.keyboard_last_toggle_time = 0.0
    node._handle_keyboard_key('\n')
    assert node.pending_target == ModeCommand.PASSIVE


def test_constant_speed_is_refreshed_until_key_expiry(control):
    node, now = control
    enable_walking(node)
    for _ in range(45):
        node._on_state(ModeState(current_state=ModeState.STANDING))
        node._handle_keyboard_key('w')
        node.timer_callback()
        assert node.cmd_vel_msg.linear.x == pytest.approx(0.3)
        now.value += 0.02
    assert node.control_mode == ModeCommand.LOCOMOTION
    assert node.cmd_vel_pub.publish.call_count == 45
    assert node.cmd_pose_pub.publish.call_count == 45
    assert node.gait_params_pub.publish.call_count == 45
    now.value += 0.41
    node._on_state(ModeState(current_state=ModeState.LOCOMOTION))
    node.timer_callback()
    assert node.cmd_vel_msg.linear.x == 0.0


@pytest.mark.parametrize(('key', 'axis', 'expected'), [
    ('w', 'x', .3), ('s', 'x', -.3), ('q', 'y', .18), ('e', 'y', -.18),
    ('a', 'yaw', .6), ('d', 'yaw', -.6),
])
def test_motion_keys(control, key, axis, expected):
    node, _ = control
    enable_walking(node)
    node._handle_keyboard_key(key)
    node.timer_callback()
    actual = {'x': node.cmd_vel_msg.linear.x, 'y': node.cmd_vel_msg.linear.y,
              'yaw': node.cmd_vel_msg.angular.z}
    assert actual[axis] == pytest.approx(expected)


def test_keys_before_confirmation_are_not_replayed(control):
    node, _ = control
    node._handle_keyboard_key('w')
    node._handle_keyboard_key(' ')
    node._handle_keyboard_key('q')
    complete(node)
    node.timer_callback()
    assert node.cmd_vel_msg.linear.x == node.cmd_vel_msg.linear.y == 0.0


def test_no_duplicate_or_queued_request_while_pending(control):
    node, now = control
    node._handle_keyboard_key(' ')
    now.value += .31
    node._handle_keyboard_key(' ')
    node._handle_keyboard_key('\n')
    assert node.command_cli.call_async.call_count == 1
    complete(node)
    node.timer_callback()
    assert node.command_cli.call_async.call_count == 1
    assert node.control_mode == ModeCommand.LOCOMOTION
    node._handle_keyboard_key(' ')
    assert node.pending_target is None  # debounce
    now.value += .31
    node._handle_keyboard_key(' ')
    assert node.pending_target == ModeCommand.STANDING


@pytest.mark.parametrize('code', [ModeResult.REJECTED, ModeResult.BUSY, ModeResult.FAILED,
                                 ModeResult.CANCELED, ModeResult.TIMEOUT, ModeResult.UNKNOWN])
def test_failed_command_does_not_enable_motion(control, code):
    node, _ = control
    node._handle_keyboard_key(' ')
    node.command_future.set_result(Command.Response(
        result=ModeResult(result=code, reason='test refusal')))
    assert node.pending_target is None
    assert not node._walking_enabled()
    assert 'test refusal' in node.command_status
    assert node.command_uncertain == (code == ModeResult.UNKNOWN)
    assert node.fsm_state.current_state == ModeState.STANDING


def test_terminal_topic_can_arrive_before_service_ack(control):
    node, _ = control
    node._handle_keyboard_key(' ')
    node._on_result(ModeResult(result=ModeResult.SUCCEEDED))
    assert not node._walking_enabled()
    node.command_future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    assert node.pending_target is None
    assert node._walking_enabled()


def test_state_loss_stops_motion_and_requires_new_permission(control):
    node, now = control
    enable_walking(node)
    node._handle_keyboard_key('w')
    now.value += 1.1
    node.timer_callback()
    assert node.cmd_vel_msg.linear.x == 0.0
    node._on_state(ModeState(current_state=ModeState.STANDING))
    node._handle_keyboard_key('w')
    node.timer_callback()
    assert node.cmd_vel_msg.linear.x == 0.0
    assert not node._walking_enabled()


def test_fault_and_reconnect_without_timer_disable_walking(control):
    node, now = control
    enable_walking(node)
    node._on_state(ModeState(current_state=ModeState.FAILURE, fault=True, fault_str='ESTOP'))
    assert not node._walking_enabled()
    assert 'ESTOP' in '\n'.join(node._status_lines())
    node._on_state(ModeState(current_state=ModeState.STANDING))
    node.control_mode = ModeCommand.LOCOMOTION
    now.value += 1.1
    node._on_state(ModeState(current_state=ModeState.STANDING))
    assert not node._walking_enabled()


def test_pending_command_lost_status_is_not_retried(control):
    node, now = control
    node._handle_keyboard_key(' ')
    old_future = node.command_future
    now.value += 1.1
    node.timer_callback()
    assert node.command_uncertain
    assert node.pending_target is None
    old_future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    node._on_state(ModeState(current_state=ModeState.STANDING))
    node._handle_keyboard_key(' ')
    assert node.command_cli.call_async.call_count == 1
    assert not node._walking_enabled()


def test_command_deadline_with_fresh_state(control):
    node, now = control
    node._handle_keyboard_key(' ')
    for _ in range(71):
        now.value += .5
        node._on_state(ModeState(current_state=ModeState.STANDING))
    node.timer_callback()
    assert node.command_uncertain
    assert node.pending_target is None
    assert 'Command result timeout' in node.command_status


def test_unavailable_service_does_not_change_mode_or_retry(control):
    node, _ = control
    node.command_cli.service_is_ready.return_value = False
    node._handle_keyboard_key(' ')
    node.timer_callback()
    node.command_cli.call_async.assert_not_called()
    assert node.control_mode == ModeCommand.STANDING


def test_settings_bounds_filter_and_combined_motion(control):
    node, _ = control
    enable_walking(node)
    for char in '\x1b[A\x1b[1;5A\x1b[1;2A':
        node._handle_keyboard_key(char)
    node._handle_keyboard_key('9')
    for key in 'wqa':
        node._handle_keyboard_key(key)
    node.timer_callback()
    assert node.cmd_vel_msg.linear.x == pytest.approx(.9)
    assert node.cmd_vel_msg.linear.y == pytest.approx(.54)
    assert node.cmd_vel_msg.angular.z == pytest.approx(1.8)
    assert node.cmd_pose_msg.linear.z == pytest.approx(.206 + .2 * .002)
    assert node.gait_params_msg.t_sw == pytest.approx(.19)
    assert node.gait_params_msg.stride_height == pytest.approx(.045)
    assert node.gait_params_msg.t_st == pytest.approx(.2)
    assert list(node.gait_params_msg.gait_offsets) == pytest.approx([0., .5, .5, 0.])
    for _ in range(200):
        node._handle_escape_sequence('\x1b[B')
        node._handle_escape_sequence('\x1b[1;5B')
        node._handle_escape_sequence('\x1b[1;2B')
    assert node.keyboard_body_z == pytest.approx(.1)
    assert node.keyboard_t_sw == pytest.approx(.01)
    assert node.keyboard_stride_height == 0.0
    for _ in range(200):
        node._handle_escape_sequence('\x1b[A')
    assert node.keyboard_body_z == pytest.approx(.26)
    node._handle_keyboard_key('1')
    assert node._t_st_for_speed() == pytest.approx(.5)


@pytest.mark.parametrize('scheme', range(4))
def test_shift_digits_send_kinematics_without_changing_speed(control, scheme):
    node, _ = control
    node._handle_keyboard_key('!@#$'[scheme])
    node.command_cli.call_async.assert_called_once()
    command = node.command_cli.call_async.call_args.args[0].command
    assert (command.command, command.target, command.params) == (
        ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS, scheme)
    assert node.keyboard_speed == .3
    assert 'CHANGE_KINEMATICS' in node._pending_name()
    complete(node)
    assert node.pending_target is None
    assert node.control_mode == ModeCommand.STANDING
    assert not node._walking_enabled()


@pytest.mark.parametrize('state', [ModeState.PASSIVE, ModeState.LOCOMOTION, ModeState.ACTION,
                                 ModeState.STANDUP, ModeState.LAYDOWN, ModeState.FAILURE])
def test_kinematics_requires_standing(control, state):
    node, _ = control
    node._on_state(ModeState(current_state=state))
    node._handle_keyboard_key('!')
    node.command_cli.call_async.assert_not_called()
    assert node.pending_target is None


def test_action_waits_for_final_result_and_blocks_other_commands(control):
    node, now = control
    node._handle_keyboard_key('!')
    node.command_future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    node._on_state(ModeState(current_state=ModeState.ACTION))
    assert node.pending_target is not None
    assert not node._walking_enabled()
    now.value += .4
    node._handle_keyboard_key('@')
    node._handle_keyboard_key('\n')
    node.command_cli.call_async.assert_called_once()
    node._on_state(ModeState(current_state=ModeState.STANDING))
    node._on_result(ModeResult(result=ModeResult.SUCCEEDED))
    assert node.control_mode == ModeCommand.STANDING
    node._handle_keyboard_key(' ')
    assert node.pending_command.command == ModeCommand.SET_MODE


def test_kinematics_failure_before_ack_is_reported(control):
    node, _ = control
    node._handle_keyboard_key('!')
    node._on_result(ModeResult(result=ModeResult.FAILED, reason='Action failed'))
    node.command_future.set_result(Command.Response(result=ModeResult(result=ModeResult.ACCEPTED)))
    assert node.pending_target is None
    assert 'Action failed' in node.command_status
    assert not node._walking_enabled()


def test_plain_digits_and_escape_do_not_start_action(control):
    node, _ = control
    for key in '1234\x1b':
        node._handle_keyboard_key(key)
    node.command_cli.call_async.assert_not_called()
    assert node.keyboard_speed == .4


def test_kinematics_key_autorepeat_is_debounced(control):
    node, now = control
    node._handle_keyboard_key('!')
    complete(node)
    now.value += .1
    node._handle_keyboard_key('!')
    node.command_cli.call_async.assert_called_once()
