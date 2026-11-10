"""ROS callbacks and LCM polling run on one executor thread."""
import math
import os
from pathlib import Path
import struct
import time
import yaml

import lcm
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from mors_ros_msgs.action import RunAction
from mors_ros_msgs.msg import ModeCommand, ModeResult, ModeState
from mors_ros_msgs.srv import Command

from .lcm_types.mors_msgs import (
    gait_params_msg, robot_cmd_msg, robot_mode_cmd_msg,
    robot_mode_result_msg, robot_mode_state_msg,
)
from .protocol import Protocol, finite_vector


def vector(twist):
    return [twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z]


class Bridge(Node):
    def __init__(self):
        super().__init__('ros2lcm_bridge')
        self.declare_parameter('lcm_url', os.environ.get('LCM_LOCOMOTION_URL', ''))
        for name, value in [('ack_timeout', 2.),
                            ('result_timeout', 30.), ('state_timeout', 1.),
                            ('poll_period', .005), ('stamped_velocity_max_age', .4),
                            ('velocity_timeout', .5)]:
            self.declare_parameter(name, value)
        self.declare_parameter('legacy_services', True)
        self.declare_parameter('active_legs', [True] * 4)
        self.declare_parameter('adaptation_type', 2)
        value = lambda name: self.get_parameter(name).value
        timing = {name: value(name) for name in (
            'ack_timeout', 'result_timeout', 'state_timeout')}
        if any(not math.isfinite(v) or v <= 0 for v in
               [*timing.values(), value('poll_period'), value('stamped_velocity_max_age'),
                value('velocity_timeout')]):
            raise ValueError('Bridge timeouts must be positive finite numbers')
        if timing['result_timeout'] < timing['ack_timeout']:
            raise ValueError('result_timeout must be >= ack_timeout')
        config_dir = os.environ.get('CONFIGPATH')
        if not config_dir:
            raise ValueError('CONFIGPATH must be set; source ~/.bashrc')
        with (Path(config_dir) / 'channels.yaml').open() as stream:
            self.channels = yaml.safe_load(stream)
        for key in ('robot_mode_cmd', 'robot_mode_state', 'robot_mode_result', 'robot_cmd', 'gait_params'):
            if not isinstance(self.channels[key], str) or not self.channels[key]:
                raise ValueError(f'Expected a nonempty LCM channel: {key}')
        self.active_legs = value('active_legs')
        self.adaptation = value('adaptation_type')
        if len(self.active_legs) != 4 or self.adaptation not in (0, 1, 2):
            raise ValueError('Expected four active_legs and adaptation_type in [0,2]')
        self.max_stamp_age = value('stamped_velocity_max_age')
        self.last_velocity_stamp = -1
        self.pose = [0.] * 6
        self.desired_velocity = [0.] * 6
        self.velocity_received = None
        self.velocity_timeout = value('velocity_timeout')
        url = value('lcm_url') or os.environ.get('LCM_LOCOMOTION_URL', '')
        if not url:
            raise ValueError('LCM_LOCOMOTION_URL must be set; source ~/.bashrc or set lcm_url')
        self.bus = lcm.LCM(url)
        self.state_pub = self.create_publisher(ModeState, 'fsm/state', 10)
        self.result_pub = self.create_publisher(ModeResult, 'fsm/result', 128)
        self.protocol = Protocol(robot_mode_cmd_msg,
                                 lambda msg: self.send('robot_mode_cmd', msg),
                                 self.publish_result, **timing)
        for channel, kind, callback in [
                ('robot_mode_state', robot_mode_state_msg, self.on_state),
                ('robot_mode_result', robot_mode_result_msg, self.protocol.on_result)]:
            def receive(_, data, kind=kind, callback=callback):
                try:
                    callback(kind.decode(data))
                except (ValueError, struct.error) as error:
                    self.get_logger().warning(f'Invalid LCM packet: {error}', throttle_duration_sec=1.)
            self.bus.subscribe(self.channels[channel], receive).set_queue_capacity(128)
        self.goals = {}
        self.group = ReentrantCallbackGroup()
        self.command_service = self.create_service(Command, 'fsm/command', self.command,
                                                   callback_group=self.group)
        self.action = ActionServer(self, RunAction, 'fsm/run_action', self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=self.cancel,
                                   callback_group=self.group)
        if value('legacy_services'):
            self.mode_service = self.create_service(RobotCmd, 'robot_mode', self.legacy_mode,
                                                    callback_group=self.group)
            self.legacy_action_service = self.create_service(
                RobotCmd, 'robot_action', self.legacy_action, callback_group=self.group)
        # Volatile depth one prevents a backlog of old streaming setpoints.
        stream_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.VOLATILE,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.velocity_sub = self.create_subscription(Twist, 'cmd_vel', self.velocity, stream_qos)
        self.stamped_sub = self.create_subscription(
            TwistStamped, 'cmd_vel_stamped', self.stamped_velocity, stream_qos)
        self.pose_sub = self.create_subscription(Twist, 'cmd_pose', self.on_pose, stream_qos)
        self.gait_sub = self.create_subscription(GaitParams, 'gait_params', self.gait, stream_qos)
        # Connection deadlines continue even if ROS simulated time is paused.
        self.poll_timer = self.create_timer(value('poll_period'), self.poll,
                                            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def send(self, channel, message):
        self.bus.publish(self.channels[channel], message.encode())

    def poll(self):
        for _ in range(128):
            if not self.bus.handle_timeout(0):
                break
        self.protocol.tick()
        for record in list(self.goals.values()):
            barrier = record.get('cancel_barrier')
            if barrier is not None and not barrier.done() and not record['cancel_inflight']:
                barrier.set_result(None)

    def publish_result(self, outcome):
        self.result_pub.publish(self.ros_result(outcome))

    def on_state(self, wire):
        if not self.protocol.on_state(wire):
            return
        msg = ModeState()
        for name in msg.get_fields_and_field_types():
            setattr(msg, name, getattr(wire, name))
        self.state_pub.publish(msg)
        for record in list(self.goals.values()):
            goal = record.get('handle')
            if goal is not None and goal.is_active:
                goal.publish_feedback(RunAction.Feedback(state=msg))

    async def command(self, request, response):
        msg = request.command
        pending = self.protocol.command(msg.command, msg.target)
        outcome = await pending.first
        response.result = self.ros_result(outcome)
        return response

    @staticmethod
    def ros_result(outcome):
        return ModeResult(result=outcome.result, reason=outcome.reason)

    async def accept_goal(self, goal):
        pending = self.protocol.start_action(goal.action_id)
        outcome = await pending.first
        if outcome.result not in (ModeResult.ACCEPTED, ModeResult.SUCCEEDED):
            return GoalResponse.REJECT
        self.goals[id(goal)] = {'request': pending, 'cancel_inflight': False}
        return GoalResponse.ACCEPT

    async def execute(self, goal):
        record = self.goals[id(goal.request)]
        record['handle'] = goal
        outcome = await record['request'].final
        if record['cancel_inflight']:
            record['cancel_barrier'] = Future()
            await record['cancel_barrier']
        if outcome.result == ModeResult.SUCCEEDED:
            goal.succeed()
        elif outcome.result == ModeResult.CANCELED and goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        self.goals.pop(id(goal.request), None)
        return RunAction.Result(result=self.ros_result(outcome))

    async def cancel(self, goal):
        record = self.goals.get(id(goal.request))
        if record is None or record['cancel_inflight']:
            return CancelResponse.REJECT
        record['cancel_inflight'] = True
        pending = self.protocol.cancel_action()
        outcome = await pending.first
        record['cancel_inflight'] = False
        if outcome.result in (ModeResult.ACCEPTED, ModeResult.SUCCEEDED):
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    async def legacy_mode(self, request, response):
        modes = {0: ModeCommand.PASSIVE, 1: ModeCommand.LOCOMOTION,
                 2: ModeCommand.BALANCE, 3: ModeCommand.BALANCE}
        if request.data not in modes:
            response.success = 0
        else:
            pending = self.protocol.command(ModeCommand.SET_MODE, modes[request.data])
            response.success = self.legacy_response(await pending.first)
        return response

    async def legacy_action(self, request, response):
        modes = {1: ModeCommand.BALANCE, 2: ModeCommand.PASSIVE}
        if request.data not in modes:
            response.success = 0
        else:
            pending = self.protocol.command(ModeCommand.SET_MODE, modes[request.data])
            response.success = self.legacy_response(await pending.first)
        return response

    @staticmethod
    def legacy_response(outcome):
        if outcome.result == ModeResult.UNKNOWN:
            return -1
        return int(outcome.result in (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))

    def velocity(self, twist):
        values = vector(twist)
        if not finite_vector(values):
            return
        self.desired_velocity = values
        self.velocity_received = time.monotonic()
        self.publish_robot_command()

    def publish_robot_command(self):
        fresh = (self.velocity_received is not None and
                 time.monotonic() - self.velocity_received <= self.velocity_timeout)
        wire = robot_cmd_msg()
        wire.cmd_vel = self.desired_velocity if fresh else [0.] * 6
        wire.cmd_pose = self.pose
        wire.active_legs, wire.adaptation_type = self.active_legs, self.adaptation
        self.send('robot_cmd', wire)

    def stamped_velocity(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or stamp <= self.last_velocity_stamp or not 0 <= age <= self.max_stamp_age:
            return
        if not finite_vector(vector(msg.twist)):
            return
        self.last_velocity_stamp = stamp
        self.velocity(msg.twist)

    def on_pose(self, twist):
        values = vector(twist)
        if not finite_vector(values):
            return
        self.pose = values
        self.publish_robot_command()

    def gait(self, msg):
        values = [msg.t_sw, msg.t_st, msg.stride_height, *msg.gait_offsets]
        if not finite_vector(values) or msg.t_sw <= 0 or msg.t_st <= 0 or msg.stride_height < 0:
            return
        if any(not 0 <= phase < 1 for phase in msg.gait_offsets):
            return
        wire = gait_params_msg()
        wire.t_sw, wire.t_st, wire.stride_height = msg.t_sw, msg.t_st, msg.stride_height
        wire.gait_type = list(msg.gait_offsets)
        wire.standing = msg.standing  # FSM ignores this legacy field for mode selection.
        self.send('gait_params', wire)

    def destroy_node(self):
        self.action.destroy()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # SIGTERM can invalidate the context while the executor creates a wait set.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
