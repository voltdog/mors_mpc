#!/usr/bin/env python3
"""Separate ROS bridge + C++ FSM processes, real ROS 2 and UDP multicast."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
import yaml
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault('ROS_DOMAIN_ID', '173')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

import lcm
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist, TwistStamped
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from mors_ros_msgs.action import RunAction
from mors_ros_msgs.msg import ModeResult, ModeState
from mors_ros_msgs.srv import Command
from ros2lcm_bridge_runtime.node import Bridge
from ros2lcm_bridge_runtime.lcm_types.mors_msgs import (
    robot_cmd_msg, robot_mode_cmd_msg, robot_mode_state_msg,
)


class BridgeStreams(unittest.TestCase):
    """Exercise stream callbacks with controlled monotonic and ROS clocks."""
    def setUp(self):
        self.packets = []
        self.bridge = SimpleNamespace(
            pose=[0.] * 6, desired_velocity=[0.] * 6, velocity_received=None,
            velocity_timeout=.5, active_legs=[True, False, True, False], adaptation=1,
            max_stamp_age=.4, last_velocity_stamp=-1,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)),
            send=lambda channel, msg: self.packets.append((channel, robot_cmd_msg.decode(msg.encode()))))
        self.bridge.publish_robot_command = lambda: Bridge.publish_robot_command(self.bridge)
        self.bridge.velocity = lambda msg: Bridge.velocity(self.bridge, msg)
        self.clock = patch('ros2lcm_bridge_runtime.node.time.monotonic', return_value=10.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_each_input_publishes_one_complete_packet(self):
        pose = Twist(); pose.linear.z = .25; pose.angular.x = .125
        Bridge.on_pose(self.bridge, pose)
        channel, packet = self.packets[-1]
        self.assertEqual(channel, 'robot_cmd')
        self.assertEqual(packet.cmd_vel, (0.,) * 6)
        self.assertEqual(packet.cmd_pose, (0., 0., .25, .125, 0., 0.))
        velocity = Twist(); velocity.linear.x = .5; velocity.angular.z = .25
        Bridge.velocity(self.bridge, velocity)
        Bridge.on_pose(self.bridge, pose)
        self.assertEqual(len(self.packets), 3)
        for _, packet in self.packets[1:]:
            self.assertEqual(packet.cmd_vel, (.5, 0., 0., 0., 0., .25))
            self.assertEqual(packet.cmd_pose, self.packets[0][1].cmd_pose)
            self.assertEqual(packet.active_legs, [True, False, True, False])
            self.assertEqual(packet.adaptation_type, 1)

    def test_velocity_expires_at_pose_update_and_resumes(self):
        velocity = Twist(); velocity.linear.x = .5
        Bridge.velocity(self.bridge, velocity)
        pose = Twist(); pose.linear.z = .25
        self.now.return_value = 10.5
        Bridge.on_pose(self.bridge, pose)
        self.assertEqual(self.packets[-1][1].cmd_vel[0], .5)
        self.now.return_value = 10.5001
        Bridge.on_pose(self.bridge, pose)
        self.assertEqual(self.packets[-1][1].cmd_vel, (0.,) * 6)
        self.assertEqual(self.bridge.velocity_received, 10.)
        self.now.return_value = 11.
        velocity.linear.x = .75
        Bridge.velocity(self.bridge, velocity)
        self.assertEqual(self.packets[-1][1].cmd_vel[0], .75)
        self.assertEqual(self.packets[-1][1].cmd_pose[2], .25)

    def test_invalid_inputs_do_not_change_setpoints_or_freshness(self):
        velocity = Twist(); velocity.linear.x = .5
        Bridge.velocity(self.bridge, velocity)
        self.now.return_value = 10.4
        for invalid in (float('nan'), float('inf'), 1e300):
            velocity.linear.x = invalid
            Bridge.velocity(self.bridge, velocity)
            pose = Twist(); pose.linear.z = invalid
            Bridge.on_pose(self.bridge, pose)
        self.assertEqual(len(self.packets), 1)
        self.assertEqual(self.bridge.velocity_received, 10.)
        self.assertEqual(self.bridge.desired_velocity[0], .5)
        self.assertEqual(self.bridge.pose, [0.] * 6)
        self.now.return_value = 10.6
        Bridge.on_pose(self.bridge, Twist())
        self.assertEqual(self.packets[-1][1].cmd_vel, (0.,) * 6)

    def test_rejected_stamps_do_not_refresh_velocity(self):
        msg = TwistStamped(); msg.twist.linear.x = .5
        for stamp in (0, 2 * 10**9, 10**8):
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            Bridge.stamped_velocity(self.bridge, msg)
        self.assertEqual(self.packets, [])
        msg.header.stamp.sec = 1; msg.header.stamp.nanosec = 0
        Bridge.stamped_velocity(self.bridge, msg)
        self.now.return_value = 10.6
        Bridge.stamped_velocity(self.bridge, msg)
        self.assertEqual(len(self.packets), 1)
        self.assertEqual(self.bridge.velocity_received, 10.)
        Bridge.on_pose(self.bridge, Twist())
        self.assertEqual(self.packets[-1][1].cmd_vel, (0.,) * 6)


@unittest.skipUnless(os.environ.get('FSM_BINARY'), 'Set FSM_BINARY to the built fsm_lcm')
class BridgeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.processes, self.logs = [], []
        self.addCleanup(self.cleanup)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
            port.bind(('127.0.0.1', 0))
            number = port.getsockname()[1]
        self.url = f'udpm://239.255.76.67:{number}?ttl=0'
        self.namespace = f'/bridge_test_{number}'
        self.config_dir = tempfile.TemporaryDirectory()
        self.channels = yaml.safe_load((Path(os.environ['CONFIGPATH']) / 'channels.yaml').read_text())
        # Произвольные имена проверяют загрузку общего YAML обеими сторонами.
        self.channels.update(robot_mode_cmd='MODE_REQUEST', robot_mode_state='MODE_STATUS',
                             robot_mode_result='MODE_REPLY', robot_cmd='MOTION_SETPOINT',
                             gait_params='GAIT_SETTINGS')
        (Path(self.config_dir.name) / 'channels.yaml').write_text(yaml.safe_dump(self.channels))
        self.env = {**os.environ, 'CONFIGPATH': self.config_dir.name}
        self.bus = lcm.LCM(self.url)
        self.packets, self.unsafe, self.states, self.results = [], [], [], []
        self.commands = []
        self.lcm_states = []
        self.bus.subscribe(self.channels['robot_cmd'], lambda _, data: self.packets.append(robot_cmd_msg.decode(data)))
        self.bus.subscribe(self.channels['robot_mode_cmd'],
                           lambda _, data: self.commands.append(robot_mode_cmd_msg.decode(data)))
        self.bus.subscribe(self.channels['robot_mode_state'],
                           lambda _, data: self.lcm_states.append(robot_mode_state_msg.decode(data)))
        self.bus.subscribe(f"({self.channels['servo_cmd']}|{self.channels['enable']})",
                           lambda channel, _: self.unsafe.append(channel))
        self.node = rclpy.create_node('bridge_test_client', namespace=self.namespace)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.state_sub = self.node.create_subscription(ModeState, 'fsm/state', self.states.append, 10)
        self.result_sub = self.node.create_subscription(ModeResult, 'fsm/result', self.results.append, 128)
        self.client = self.node.create_client(Command, 'fsm/command')
        self.action = ActionClient(self.node, RunAction, 'fsm/run_action')
        self.vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.node.create_publisher(Twist, 'cmd_pose', 10)
        self.stamped_pub = self.node.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.gait_pub = self.node.create_publisher(GaitParams, 'gait_params', 10)
        self.fsm = self.start_fsm()
        executable = Path(get_package_prefix('ros2lcm_bridge')) / 'lib/ros2lcm_bridge/bridge_node'
        self.bridge = self.start([
            str(executable), '--ros-args', '-r', f'__ns:={self.namespace}', '-p', f'lcm_url:={self.url}',
            '-p', 'state_timeout:=0.5',
            '-p', 'ack_timeout:=0.8'])
        self.wait(lambda: self.states and self.client.service_is_ready() and self.action.server_is_ready())

    def start(self, args):
        log = tempfile.TemporaryFile(mode='w+t')
        process = subprocess.Popen(args, stdout=log, stderr=log, env=self.env)
        self.processes.append(process)
        self.logs.append(log)
        return process

    def start_fsm(self):
        return self.start([os.environ['FSM_BINARY'], '--stub', '--url', self.url,
                           '--stub-duration', '.4', '--status-ms', '40'])

    def cleanup(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        for process, log in zip(self.processes, self.logs):
            log.seek(0)
            if process.returncode not in (0, -15):
                print(log.read())
            log.close()
        if hasattr(self, 'node'):
            self.action.destroy()
            self.executor.shutdown()
            self.node.destroy_node()
        self.config_dir.cleanup()
        self.assertFalse(self.unsafe)

    def pump(self):
        for _ in range(128):
            if not self.bus.handle_timeout(0):
                break
        self.executor.spin_once(timeout_sec=.005)
        if hasattr(self, 'bridge') and self.bridge.poll() is not None:
            self.logs[self.processes.index(self.bridge)].seek(0)
            self.fail(self.logs[self.processes.index(self.bridge)].read())

    def wait(self, condition, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if condition():
                return
        self.fail('Timed out waiting for ROS/LCM event')

    def run_for(self, duration, callback=lambda: None):
        deadline = time.monotonic() + duration
        next_publish = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                callback()
                next_publish = time.monotonic() + .02
            self.pump()

    def complete(self, future):
        self.wait(future.done)
        return future.result()

    def command(self, code=0, target=2):
        request = Command.Request()
        request.command.command, request.command.target = code, target
        return self.complete(self.client.call_async(request)).result

    def balance(self):
        result = self.command()
        self.assertIn(result.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        self.wait(lambda: self.states[-1].current_state == ModeState.BALANCE)

    def goal(self, action_id=0, feedback=None):
        return self.complete(self.action.send_goal_async(
            RunAction.Goal(action_id=action_id), feedback_callback=feedback))

    def test_command_ack_completion_and_legacy(self):
        initial = self.command()
        self.assertIn(initial.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        self.wait(lambda: any(r.result == ModeResult.SUCCEEDED for r in self.results))
        repeated = self.command()
        self.assertIn(repeated.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        invalid = self.command(3, target=255)
        self.assertEqual(invalid.result, ModeResult.REJECTED)
        client = self.node.create_client(RobotCmd, 'robot_action')
        self.wait(client.service_is_ready)
        response = self.complete(client.call_async(RobotCmd.Request(data=2)))
        self.assertEqual(response.success, 1)
        self.wait(lambda: self.states[-1].current_state == ModeState.PASSIVE)

    def test_numeric_algorithm_selection_and_external_rejection(self):
        selected = self.command(3, target=0)
        self.assertEqual(selected.result, ModeResult.SUCCEEDED)
        self.wait(lambda: self.states[-1].algorithm_id == 0)
        unknown = self.command(3, target=255)
        self.assertEqual(unknown.result, ModeResult.REJECTED)
        self.wait(lambda: any(c.command == 3 and c.target == 255 for c in self.commands))
        self.assertEqual(self.states[-1].algorithm_id, 0)
        self.assertEqual(self.states[-1].current_state, ModeState.PASSIVE)
        external = self.command(target=5)
        self.assertEqual(external.result, ModeResult.REJECTED)
        self.assertEqual(external.reason, 'INVALID_COMMAND_OR_MODE')
        self.run_for(.1)
        self.assertFalse(any(c.command == 0 and c.target == 5 for c in self.commands))

    def test_external_handover_and_return(self):
        initial = self.command(6, target=255)
        self.assertEqual(initial.result, ModeResult.ACCEPTED)
        self.wait(lambda: any(s.phase == ModeState.PHASE_TRANSFERRING for s in self.states))
        self.wait(lambda: any(r.result == ModeResult.SUCCEEDED
                             for r in self.results))
        self.wait(lambda: self.states[-1].phase == ModeState.PHASE_EXTERNAL)
        self.assertEqual(self.states[-1].current_state, ModeState.EXTERNAL)
        again = self.command(6)
        self.assertEqual(again.result, ModeResult.SUCCEEDED)
        self.assertTrue(all(c.target == 0 for c in self.commands if c.command == 6))
        self.balance()
        self.config_dir.cleanup()
        self.assertFalse(self.unsafe)

    def test_action_success_failure_rejection_and_feedback(self):
        self.balance()
        feedback = []
        goal = self.goal(feedback=feedback.append)
        self.assertTrue(goal.accepted)
        response = self.complete(goal.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.result.result, ModeResult.SUCCEEDED)
        self.assertTrue(feedback)
        goal = self.goal(1)
        self.assertTrue(goal.accepted)
        response = self.complete(goal.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.result.result, ModeResult.FAILED)
        rejected = self.goal(255)
        self.assertFalse(rejected.accepted)
        self.wait(lambda: any(r.result == ModeResult.REJECTED for r in self.results))

    def test_action_cancel_waits_for_fsm_safe_exit(self):
        self.balance()
        goal = self.goal()
        self.assertTrue(goal.accepted)
        canceled = self.complete(goal.cancel_goal_async())
        self.assertTrue(canceled.goals_canceling)
        response = self.complete(goal.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(response.result.result.result, ModeResult.CANCELED)
        self.wait(lambda: self.states[-1].current_state == ModeState.BALANCE)

    def test_estop_interrupts_action_and_clear_fault_leaves_passive(self):
        self.balance()
        goal = self.goal()
        self.assertTrue(goal.accepted)
        busy = self.goal()
        self.assertFalse(busy.accepted)
        result = self.command(4)
        self.assertIn(result.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        response = self.complete(goal.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.result.result, ModeResult.FAILED)
        self.wait(lambda: self.states[-1].fault)
        self.assertEqual(self.states[-1].current_state, ModeState.PASSIVE)
        rejected = self.command(5)
        self.assertEqual(rejected.result, ModeResult.REJECTED)
        self.run_for(1.1)
        cleared = self.command(5)
        self.assertEqual(cleared.result, ModeResult.SUCCEEDED)
        self.wait(lambda: not self.states[-1].fault)
        self.assertEqual(self.states[-1].current_state, ModeState.PASSIVE)

    def test_pose_stream_keeps_command_fresh_with_zero_expired_velocity(self):
        self.balance()
        result = self.command(target=3)
        self.assertIn(result.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        velocity = Twist(); velocity.linear.x = .12; velocity.angular.z = .2
        self.run_for(.3, lambda: self.vel_pub.publish(velocity))
        self.wait(lambda: self.states[-1].current_state == ModeState.LOCOMOTION)
        pose = Twist(); pose.linear.z = .21
        self.run_for(.8, lambda: self.pose_pub.publish(pose))
        self.assertEqual(self.packets[-1].cmd_vel, (0.,) * 6)
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .21, places=5)
        self.assertEqual(self.states[-1].current_state, ModeState.LOCOMOTION)
        velocity.linear.x = .18
        self.run_for(.15, lambda: self.vel_pub.publish(velocity))
        self.assertAlmostEqual(self.packets[-1].cmd_vel[0], .18, places=5)
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .21, places=5)
        self.run_for(.1)  # Drain the final input before checking the absence of periodic output.
        count = len(self.packets)
        gait = GaitParams(t_sw=.2, t_st=.3, stride_height=.04,
                          gait_offsets=[0., .5, .5, 0.], standing=False)
        bad = Twist(); bad.linear.x = float('nan')
        self.run_for(.9, lambda: (self.gait_pub.publish(gait), self.pose_pub.publish(bad),
                                 self.vel_pub.publish(bad)))
        self.assertEqual(len(self.packets), count)
        self.assertEqual(self.states[-1].current_state, ModeState.BALANCE)

    def test_pose_before_velocity_publishes_zero_and_preserves_pose(self):
        pose = Twist(); pose.linear.z = .24
        self.run_for(.2, lambda: self.pose_pub.publish(pose))
        self.wait(lambda: bool(self.packets))
        self.assertTrue(all(p.cmd_vel == (0.,) * 6 for p in self.packets))
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .24, places=5)
        velocity = Twist(); velocity.linear.x = .16
        self.run_for(.15, lambda: self.vel_pub.publish(velocity))
        self.assertAlmostEqual(self.packets[-1].cmd_vel[0], .16, places=5)
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .24, places=5)
        self.assertEqual(self.states[-1].current_state, ModeState.PASSIVE)

    def test_stamped_velocity_and_invalid_streams(self):
        self.run_for(.2)
        msg = TwistStamped()
        msg.twist.linear.x = .15
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.stamped_pub.publish(msg)
        self.wait(lambda: self.packets)
        count = len(self.packets)
        self.run_for(.15, lambda: self.stamped_pub.publish(msg))
        self.assertEqual(len(self.packets), count)
        msg.header.stamp.sec -= 10
        self.run_for(.1, lambda: self.stamped_pub.publish(msg))
        bad = Twist()
        bad.linear.x = float('nan')
        self.run_for(.1, lambda: self.vel_pub.publish(bad))
        bad.linear.x = 1e300
        self.run_for(.1, lambda: self.vel_pub.publish(bad))
        self.assertEqual(len(self.packets), count)

    def test_disconnect_reports_unknown_without_replaying_action(self):
        self.balance()
        goal = self.goal()
        self.assertTrue(goal.accepted)
        self.fsm.terminate()
        self.fsm.wait(timeout=3)
        response = self.complete(goal.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.result.result, ModeResult.UNKNOWN)
        self.assertEqual(sum(c.command == 1 for c in self.commands), 1)

    def test_no_fsm_reply_never_confirms_acceptance(self):
        self.fsm.terminate()
        self.fsm.wait(timeout=3)
        result = self.command(target=2)
        self.assertEqual(result.result, ModeResult.UNKNOWN)
        goal = self.goal()
        self.assertFalse(goal.accepted)

    def test_service_cannot_start_or_cancel_actions(self):
        self.balance()
        for code in (robot_mode_cmd_msg.START_ACTION, robot_mode_cmd_msg.CANCEL_ACTION):
            result = self.command(code)
            self.assertEqual(result.result, ModeResult.REJECTED)
        self.run_for(.1)
        self.assertFalse(any(c.command in (1, 2) for c in self.commands))


if __name__ == '__main__':
    unittest.main(verbosity=2)
