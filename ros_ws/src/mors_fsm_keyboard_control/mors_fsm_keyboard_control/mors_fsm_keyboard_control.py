"""Terminal teleoperation for the ROS 2 interface of FSMLocomotion."""

import math
import os
import select
import shutil
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams, ModeCommand, ModeResult, ModeState
from mors_ros_msgs.srv import Command
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


BODY_Z_MIN = 0.1
BODY_Z_MAX = 0.26
KEYBOARD_BODY_Z = 0.206
KEYBOARD_BODY_Z_STEP = 0.002
KEYBOARD_STRIDE_HEIGHT = 0.04
KEYBOARD_STRIDE_HEIGHT_STEP = 0.005
KEYBOARD_T_SW = 0.18
KEYBOARD_T_SW_STEP = 0.01
KEYBOARD_T_SW_MIN = 0.01
KEYBOARD_T_ST_MAX = 0.5
KEYBOARD_T_ST_MIN = 0.2
KEYBOARD_GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]
KEYBOARD_DEFAULT_SPEED = 0.3
KEYBOARD_MIN_SPEED = 0.1
KEYBOARD_MAX_SPEED = 0.9
KEYBOARD_COMMAND_TIMEOUT = 0.4
KEYBOARD_TOGGLE_DEBOUNCE = 0.3
CMD_POSE_LPF_ALPHA = 0.2
TIMER_PERIOD = 0.02
KINEMATICS_SCHEMES = (
    ModeCommand.KNEE_INSIDE, ModeCommand.KNEE_OUTSIDE,
    ModeCommand.KNEE_BACK, ModeCommand.KNEE_FRONT,
)
KINEMATICS_NAMES = dict(zip(KINEMATICS_SCHEMES, ('INSIDE', 'OUTSIDE', 'BACK', 'FRONT')))
KINEMATICS_KEYS = dict(zip('!@#$', KINEMATICS_SCHEMES))

STATE_NAMES = {
    getattr(ModeState, name): name
    for name in ('PASSIVE', 'STANDUP', 'STANDING', 'LOCOMOTION', 'ACTION',
                 'EXTERNAL', 'LAYDOWN', 'FAILURE')
}
RESULT_NAMES = {
    getattr(ModeResult, name): name
    for name in ('ACCEPTED', 'SUCCEEDED', 'REJECTED', 'BUSY', 'CANCELED',
                 'SUPERSEDED', 'TIMEOUT', 'FAILED', 'UNKNOWN')
}


class MorsFsmKeyboardControl(Node):
    def __init__(self):
        super().__init__('mors_fsm_keyboard_control')
        self.declare_parameter('state_timeout', 1.0)
        self.declare_parameter('command_timeout', 35.0)
        self.state_timeout = self.get_parameter('state_timeout').value
        self.command_timeout = self.get_parameter('command_timeout').value
        if any(not math.isfinite(value) or value <= 0 for value in (
                self.state_timeout, self.command_timeout)):
            raise ValueError('Timeouts must be positive finite numbers')

        self.command_cli = self.create_client(Command, 'fsm/command')
        self.state_sub = self.create_subscription(ModeState, 'fsm/state', self._on_state, 10)
        self.result_sub = self.create_subscription(ModeResult, 'fsm/result', self._on_result, 128)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_pose_pub = self.create_publisher(Twist, 'cmd_pose', 1)
        self.gait_params_pub = self.create_publisher(GaitParams, 'gait_params', 1)
        self.cmd_vel_msg = Twist()
        self.cmd_pose_msg = Twist()
        self.gait_params_msg = GaitParams()

        self.keyboard_body_z = KEYBOARD_BODY_Z
        self.filtered_body_z = KEYBOARD_BODY_Z
        self.keyboard_stride_height = KEYBOARD_STRIDE_HEIGHT
        self.keyboard_t_sw = KEYBOARD_T_SW
        self.keyboard_speed = KEYBOARD_DEFAULT_SPEED
        self.directions = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.direction_until = dict.fromkeys(self.directions, 0.0)
        self.keyboard_last_toggle_time = -math.inf
        self.keyboard_mode_last_toggle_time = -math.inf
        self.keyboard_kinematics_last_time = -math.inf
        self.keyboard_escape_sequence = ''
        self.keyboard_fd = None
        self.keyboard_term_settings = None

        self.fsm_state = None
        self.state_received_at = -math.inf
        # STANDING in fsm/state does not reveal the FSM's walking permission.
        # Require a local Space request before accepting movement keys.
        self.control_mode = ModeCommand.STANDING
        self.pending_target = None
        self.pending_command = None
        self.command_future = None
        self.command_accepted = False
        self.early_result = None
        self.command_sent_at = 0.0
        self.command_uncertain = False
        self.command_status = 'Waiting for FSM'

        self.stdout_is_tty = sys.stdout.isatty()
        self.prev_status_lines = None
        self._setup_keyboard_input()
        self._initialize_ui()
        self.timer = self.create_timer(
            TIMER_PERIOD, self.timer_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _state_is_fresh(self):
        return time.monotonic() - self.state_received_at <= self.state_timeout

    def _stop_walking(self):
        self.control_mode = ModeCommand.STANDING
        for axis in self.directions:
            self.directions[axis] = 0.0
            self.direction_until[axis] = 0.0

    def _on_state(self, msg):
        if not self._state_is_fresh():
            self._stop_walking()
            if self.pending_target is not None:
                self._finish_command(ModeResult(
                    result=ModeResult.UNKNOWN, reason='FSM status lost'))
        self.fsm_state = msg
        self.state_received_at = time.monotonic()
        if self.command_status == 'Waiting for FSM':
            self.command_status = 'Ready'
        if msg.fault or msg.current_state not in (ModeState.STANDING, ModeState.LOCOMOTION):
            self._stop_walking()

    def _request_mode(self, target):
        self._request_command(
            ModeCommand(command=ModeCommand.SET_MODE, target=target),
            (ModeState.PASSIVE, ModeState.STANDING, ModeState.LOCOMOTION))

    def _request_kinematics(self, scheme):
        self._request_command(
            ModeCommand(command=ModeCommand.START_ACTION,
                        target=ModeCommand.CHANGE_KINEMATICS, params=scheme),
            (ModeState.STANDING,))

    def _pending_name(self):
        if self.pending_command is None:
            return 'none'
        if self.pending_command.command == ModeCommand.START_ACTION:
            return 'CHANGE_KINEMATICS: ' + KINEMATICS_NAMES[self.pending_command.params]
        return STATE_NAMES[self.pending_target]

    def _request_command(self, command, allowed_states):
        if self.command_uncertain:
            self.command_status = 'UNKNOWN: restart control after checking bridge/FSM'
            return
        if self.pending_target is not None:
            self.command_status = 'Wait for the current command to finish'
            return
        if not self._state_is_fresh() or not self.command_cli.service_is_ready():
            self.command_status = 'FSM or command service unavailable'
            return
        if self.fsm_state.fault or self.fsm_state.current_state not in allowed_states:
            states = ', '.join(STATE_NAMES[state] for state in allowed_states)
            self.command_status = f'Wait for {states} without fault'
            return

        self._stop_walking()
        self.pending_target = command.target
        self.pending_command = command
        self.command_accepted = False
        self.early_result = None
        self.command_sent_at = time.monotonic()
        self.command_status = f'Requesting {self._pending_name()}'
        request = Command.Request(command=command)
        try:
            self.command_future = self.command_cli.call_async(request)
            self.command_future.add_done_callback(self._on_command_response)
        except Exception as error:
            self._finish_command(ModeResult(result=ModeResult.UNKNOWN, reason=str(error)))

    def _on_command_response(self, future):
        if future is not self.command_future:
            return
        try:
            result = future.result().result
        except Exception as error:
            self._finish_command(ModeResult(result=ModeResult.UNKNOWN, reason=str(error)))
            return
        if result.result == ModeResult.ACCEPTED:
            self.command_accepted = True
            self.command_status = f'ACCEPTED: {self._pending_name()}'
            # Topic and service deliveries can arrive in either order.
            if self.early_result is not None:
                self._finish_command(self.early_result)
        else:
            self._finish_command(result)

    def _on_result(self, msg):
        # Results have no request IDs. This node is the bridge's only command client.
        if self.pending_target is None or msg.result == ModeResult.ACCEPTED:
            return
        if self.command_accepted:
            self._finish_command(msg)
        else:
            self.early_result = msg

    def _finish_command(self, result):
        target = self.pending_target
        self._stop_walking()
        if (result.result == ModeResult.SUCCEEDED and self._state_is_fresh()
                and self.pending_command.command == ModeCommand.SET_MODE):
            self.control_mode = target
        elif result.result == ModeResult.UNKNOWN:
            # Never retry a command whose execution is uncertain.
            self.command_uncertain = True
        self.command_status = RESULT_NAMES.get(result.result, str(result.result))
        if result.reason:
            self.command_status += ': ' + ' '.join(result.reason.split())
        if self.command_future is not None and not self.command_future.done():
            self.command_cli.remove_pending_request(self.command_future)
        self.command_future = None
        self.pending_target = None
        self.pending_command = None
        self.command_accepted = False
        self.early_result = None

    def _setup_keyboard_input(self):
        if not sys.stdin.isatty():
            return
        self.keyboard_fd = sys.stdin.fileno()
        try:
            self.keyboard_term_settings = termios.tcgetattr(self.keyboard_fd)
            tty.setcbreak(self.keyboard_fd)
        except termios.error:
            self.keyboard_fd = None
            self.keyboard_term_settings = None

    def _poll_keyboard(self):
        if self.keyboard_fd is None:
            return
        while select.select([self.keyboard_fd], [], [], 0.0)[0]:
            try:
                key = os.read(self.keyboard_fd, 1)
            except OSError:
                break
            if not key:
                break
            self._handle_keyboard_key(key.decode(errors='ignore'))

    def _handle_keyboard_key(self, key):
        if not key:
            return
        if self.keyboard_escape_sequence:
            if self.keyboard_escape_sequence == '\x1b' and key != '[':
                self.keyboard_escape_sequence = ''
            else:
                self.keyboard_escape_sequence += key
                if key.isalpha() or key == '~':
                    self._handle_escape_sequence(self.keyboard_escape_sequence)
                    self.keyboard_escape_sequence = ''
                return
        if key == '\x1b':
            self.keyboard_escape_sequence = key
            return

        now = time.monotonic()
        if key in ('\r', '\n'):
            if now - self.keyboard_last_toggle_time > KEYBOARD_TOGGLE_DEBOUNCE:
                self.keyboard_last_toggle_time = now
                passive = (self.fsm_state is not None
                           and self.fsm_state.current_state == ModeState.PASSIVE)
                target = ModeCommand.STANDING if passive else ModeCommand.PASSIVE
                self._request_mode(target)
        elif key == ' ':
            if now - self.keyboard_mode_last_toggle_time > KEYBOARD_TOGGLE_DEBOUNCE:
                self.keyboard_mode_last_toggle_time = now
                if self.fsm_state is not None and self.fsm_state.current_state in (
                        ModeState.STANDING, ModeState.LOCOMOTION):
                    target = (ModeCommand.STANDING if self.control_mode == ModeCommand.LOCOMOTION
                              else ModeCommand.LOCOMOTION)
                    self._request_mode(target)
        elif key.lower() in ('w', 's', 'q', 'e', 'a', 'd'):
            axis, direction = {
                'w': ('x', 1.0), 's': ('x', -1.0), 'q': ('y', 1.0),
                'e': ('y', -1.0), 'a': ('yaw', 1.0), 'd': ('yaw', -1.0),
            }[key.lower()]
            if self._walking_enabled():
                self.directions[axis] = direction
                self.direction_until[axis] = now + KEYBOARD_COMMAND_TIMEOUT
        elif key in '123456789':
            self.keyboard_speed = int(key) / 10.0
        elif key in KINEMATICS_KEYS:
            if now - self.keyboard_kinematics_last_time > KEYBOARD_TOGGLE_DEBOUNCE:
                self.keyboard_kinematics_last_time = now
                self._request_kinematics(KINEMATICS_KEYS[key])

    def _handle_escape_sequence(self, sequence):
        if not sequence.startswith('\x1b[') or sequence[-1] not in ('A', 'B'):
            return
        params = sequence[2:-1]
        try:
            modifier = int(params.split(';')[-1]) if ';' in params else 1
        except ValueError:
            modifier = 1
        direction = 1.0 if sequence[-1] == 'A' else -1.0
        if modifier in (5, 6, 7, 8):
            self.keyboard_t_sw = max(
                KEYBOARD_T_SW_MIN, self.keyboard_t_sw + direction * KEYBOARD_T_SW_STEP)
        elif modifier in (2, 4, 6, 8):
            self.keyboard_stride_height = max(
                0.0, self.keyboard_stride_height + direction * KEYBOARD_STRIDE_HEIGHT_STEP)
        else:
            self.keyboard_body_z = min(BODY_Z_MAX, max(
                BODY_Z_MIN, self.keyboard_body_z + direction * KEYBOARD_BODY_Z_STEP))

    def _t_st_for_speed(self):
        speed = min(KEYBOARD_MAX_SPEED, max(KEYBOARD_MIN_SPEED, self.keyboard_speed))
        ratio = (speed - KEYBOARD_MIN_SPEED) / (KEYBOARD_MAX_SPEED - KEYBOARD_MIN_SPEED)
        return KEYBOARD_T_ST_MAX + ratio * (KEYBOARD_T_ST_MIN - KEYBOARD_T_ST_MAX)

    def _walking_enabled(self):
        return (self._state_is_fresh() and not self.fsm_state.fault
                and self.fsm_state.current_state in (ModeState.STANDING, ModeState.LOCOMOTION)
                and self.control_mode == ModeCommand.LOCOMOTION
                and self.pending_target is None and not self.command_uncertain)

    def timer_callback(self):
        now = time.monotonic()
        if not self._state_is_fresh():
            self._stop_walking()
            if self.pending_target is not None:
                self._finish_command(ModeResult(
                    result=ModeResult.UNKNOWN, reason='FSM status lost'))
        if self.pending_target is not None and now - self.command_sent_at > self.command_timeout:
            self._finish_command(ModeResult(
                result=ModeResult.UNKNOWN, reason='Command result timeout'))
        self._poll_keyboard()

        self.cmd_vel_msg = Twist()
        if self._walking_enabled():
            for axis in self.directions:
                if now > self.direction_until[axis]:
                    self.directions[axis] = 0.0
            self.cmd_vel_msg.linear.x = self.directions['x'] * self.keyboard_speed
            self.cmd_vel_msg.linear.y = self.directions['y'] * self.keyboard_speed * 0.6
            self.cmd_vel_msg.angular.z = self.directions['yaw'] * self.keyboard_speed * 2.0

        self.filtered_body_z += CMD_POSE_LPF_ALPHA * (self.keyboard_body_z - self.filtered_body_z)
        self.cmd_pose_msg.linear.z = self.filtered_body_z
        self.gait_params_msg.standing = self.control_mode != ModeCommand.LOCOMOTION
        self.gait_params_msg.stride_height = self.keyboard_stride_height
        self.gait_params_msg.t_sw = self.keyboard_t_sw
        self.gait_params_msg.t_st = self._t_st_for_speed()
        self.gait_params_msg.gait_offsets = KEYBOARD_GAIT_TYPE
        # Refresh all setpoints, including unchanged values and after bridge restarts.
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)
        self._render_status()

    def _style(self, text, code):
        return f'\x1b[{code}m{text}\x1b[0m' if self.stdout_is_tty else text

    def _status_lines(self):
        connected = self._state_is_fresh()
        state = STATE_NAMES.get(self.fsm_state.current_state, '?') if connected else 'UNAVAILABLE'
        fault = self.fsm_state.fault_str if connected and self.fsm_state.fault else 'none'
        lines = [
            f'FSM state: {state}',
            f'control mode: {STATE_NAMES[self.control_mode]}',
            f'pending target: {self._pending_name()}',
            f'command: {self.command_status}',
            f'fault: {" ".join(fault.split())}',
            f'v_x max: {self.keyboard_speed:.1f} m/s',
            f'v_y max: {self.keyboard_speed * 0.6:.2f} m/s',
            f'yaw rate max: {self.keyboard_speed * 2.0:.1f} rad/s',
            f'robot height: {self.keyboard_body_z:.3f} m',
            f'stride height: {self.keyboard_stride_height:.3f} m',
            f't_sw: {self.keyboard_t_sw:.2f} s',
            f't_st: {self._t_st_for_speed():.4f} s',
        ]
        if self.stdout_is_tty:
            width = max(1, shutil.get_terminal_size().columns - 1)
            lines = [line[:width] for line in lines]
        return lines

    def _write_plain(self, text):
        sys.stdout.write(text)
        sys.stdout.flush()

    def _initialize_ui(self):
        lines = [
            self._style('=' * 60, '90'),
            self._style('mors_fsm_keyboard_control', '1;96'),
            'Enter: stand up / lay down',
            'Space: toggle STANDING / LOCOMOTION control',
            'W/S: forward/backward | Q/E: left/right | A/D: rotate CCW/CW',
            'Up/Down: body height +/-0.002 m',
            'Ctrl+Up/Down: swing duration +/-0.01 s',
            'Shift+Up/Down: stride height +/-0.005 m',
            '1..9: speed 0.1..0.9 | Ctrl+C: exit',
            'Shift+1..4 (! @ # $): knees INSIDE / OUTSIDE / BACK / FRONT',
            self._style('=' * 60, '90'),
            *self._status_lines(),
        ]
        if self.stdout_is_tty:
            self._write_plain('\x1b[2J\x1b[H\x1b[?25l')
        self._write_plain('\n'.join(lines) + '\n')
        self.prev_status_lines = self._status_lines()

    def _render_status(self):
        lines = self._status_lines()
        if lines == self.prev_status_lines:
            return
        if self.stdout_is_tty:
            self._write_plain(f'\x1b[{len(lines)}F')
            for line in lines:
                self._write_plain(f'\x1b[2K{line}\n')
        else:
            self._write_plain('\n'.join(lines) + '\n')
        self.prev_status_lines = lines

    def destroy_node(self):
        try:
            if self.context.ok():
                self.cmd_vel_pub.publish(Twist())
        finally:
            if self.keyboard_fd is not None and self.keyboard_term_settings is not None:
                try:
                    termios.tcsetattr(
                        self.keyboard_fd, termios.TCSADRAIN, self.keyboard_term_settings)
                except termios.error:
                    pass
            if self.stdout_is_tty:
                self._write_plain('\x1b[?25h\n')
        return super().destroy_node()


def main(args=None):
    # Keep the context alive for the final zero command on Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MorsFsmKeyboardControl()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
