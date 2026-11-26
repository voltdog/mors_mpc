#!/usr/bin/env python3
"""Separate ROS bridge + C++ FSM processes, real ROS 2 and UDP multicast."""
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
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
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist, TwistStamped
from mors_ros_msgs.msg import GaitParams, ModeCommand, ModeResult, ModeState
from mors_ros_msgs.srv import Command, RobotCmd
from rclpy.executors import SingleThreadedExecutor
from ros2lcm_bridge_runtime.node import Bridge
from ros2lcm_bridge_runtime.lcm_types.mors_msgs import (
    gait_params_msg, robot_cmd_msg, robot_mode_cmd_msg,
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


@unittest.skipUnless(os.environ.get('FSM_BINARY'), 'Set FSM_BINARY to fsm_locomotion_node')
class BridgeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bindings = tempfile.TemporaryDirectory()
        cls.addClassCleanup(bindings.cleanup)
        schema = Path(__file__).resolve().parents[4] / 'lcm_msgs' / 'robot_state_msg.lcm'
        subprocess.run(['lcm-gen', '-p', '--ppath', bindings.name,
                        str(schema), str(schema.with_name('mpc_cmd_msg.lcm'))], check=True)
        sys.path.insert(0, bindings.name)
        try:
            from mors_msgs import robot_state_msg, mpc_cmd_msg
            cls.robot_state_type = robot_state_msg
            cls.mpc_command_type = mpc_cmd_msg
        finally:
            sys.path.remove(bindings.name)
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
        for name in ('channels.yaml', 'robot.yaml', 'timesteps.yaml', 'heightmap_builder.yaml', 'locomotion_controller.yaml'):
            shutil.copy(Path(os.environ['CONFIGPATH']) / name, self.config_dir.name)
        channels_path = Path(self.config_dir.name) / 'channels.yaml'
        self.channels = yaml.safe_load(channels_path.read_text())
        # Произвольные имена проверяют загрузку общего YAML обеими сторонами.
        self.channels.update(robot_mode_cmd='MODE_REQUEST', robot_mode_state='MODE_STATUS',
                             robot_mode_result='MODE_REPLY', robot_cmd='MOTION_SETPOINT',
                             gait_params='GAIT_SETTINGS', robot_state='ROBOT_TELEMETRY')
        channels_path.write_text(yaml.safe_dump(self.channels))
        robot_path = Path(self.config_dir.name) / 'robot.yaml'
        robot_config = yaml.safe_load(robot_path.read_text())
        robot_config['robot_config']['debug_mode'] = True
        robot_path.write_text(yaml.safe_dump(robot_config))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
            port.bind(('127.0.0.1', 0))
            servo_url = f'udpm://239.255.76.68:{port.getsockname()[1]}?ttl=0'
        self.env = {**os.environ, 'LCM_SERVO_URL': servo_url, 'CONFIGPATH': self.config_dir.name,
                    'LCM_LOCOMOTION_URL': '', 'LCM_DEFAULT_URL': self.url}
        self.bus = lcm.LCM(self.url)
        self.packets, self.unsafe, self.states, self.results = [], [], [], []
        self.commands, self.gait_packets = [], []
        self.follow_body_height = False
        self.bus.subscribe(self.channels['mpc_cmd'], self.follow_height_reference)
        self.bus.subscribe(self.channels['robot_cmd'], lambda _, data: self.packets.append(robot_cmd_msg.decode(data)))
        self.bus.subscribe(self.channels['robot_mode_cmd'],
                           lambda _, data: self.commands.append(robot_mode_cmd_msg.decode(data)))
        self.bus.subscribe(self.channels['gait_params'],
                           lambda _, data: self.gait_packets.append(gait_params_msg.decode(data)))
        self.bus.subscribe(f"({self.channels['servo_cmd']}|{self.channels['enable']})",
                           lambda channel, _: self.unsafe.append(channel))
        self.node = rclpy.create_node('bridge_test_client', namespace=self.namespace)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.state_sub = self.node.create_subscription(ModeState, 'fsm/state', self.states.append, 10)
        self.result_sub = self.node.create_subscription(ModeResult, 'fsm/result', self.results.append, 128)
        self.client = self.node.create_client(Command, 'fsm/command')
        self.vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.node.create_publisher(Twist, 'cmd_pose', 10)
        self.stamped_pub = self.node.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.gait_pub = self.node.create_publisher(GaitParams, 'gait_params', 10)
        self.telemetry = self.robot_state_type()
        self.telemetry.body.orientation_quaternion = [0., 0., 0., 1.]
        self.telemetry.body.position[2] = .20
        self.telemetry.joints.position = [0., -1.57, 2.9, 0., 1.57, -2.9,
                                         0., 1.57, -2.9, 0., -1.57, 2.9]
        self.telemetry.legs.r1_pos = [.18, -.13, 0.]
        self.telemetry.legs.l1_pos = [.18, .13, 0.]
        self.telemetry.legs.r2_pos = [-.18, -.13, 0.]
        self.telemetry.legs.l2_pos = [-.18, .13, 0.]
        self.telemetry.legs.contact_states = [True] * 4
        self.telemetry.readiness = [True] * 18 + [False]
        self.streaming = False
        self.next_telemetry = 0.
        self.fsm = self.start([os.environ['FSM_BINARY']])
        executable = Path(get_package_prefix('ros2lcm_bridge')) / 'lib/ros2lcm_bridge/bridge_node'
        self.bridge = self.start([
            str(executable), '--ros-args', '-r', f'__ns:={self.namespace}',
            '-p', 'state_timeout:=0.5',
            '-p', 'ack_timeout:=0.8'])
        self.wait(lambda: self.states and self.client.service_is_ready())

    def start(self, args):
        log = tempfile.TemporaryFile(mode='w+t')
        process = subprocess.Popen(args, stdout=log, stderr=log, env=self.env)
        self.processes.append(process)
        self.logs.append(log)
        return process

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
            self.executor.shutdown()
            self.node.destroy_node()
        if hasattr(self, 'config_dir'):
            self.config_dir.cleanup()
        self.assertFalse(getattr(self, 'unsafe', []))

    def pump(self):
        now = time.monotonic()
        if self.streaming and now >= self.next_telemetry:
            self.bus.publish(self.channels['robot_state'], self.telemetry.encode())
            self.next_telemetry = now + .005
        for _ in range(128):
            if not self.bus.handle_timeout(0):
                break
        self.executor.spin_once(timeout_sec=.005)
        for process, log in zip(self.processes, self.logs):
            if process.poll() not in (None, 0):
                log.seek(0)
                self.fail(log.read())

    def follow_height_reference(self, _channel, data):
        if self.follow_body_height:
            command = self.mpc_command_type.decode(data)
            self.telemetry.body.position[2] = command.cmd_pose[2]
            self.telemetry.body.lin_vel[2] = command.cmd_vel[2]

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

    def command(self, code=0, target=2, params=0):
        request = Command.Request()
        request.command.command, request.command.target = code, target
        request.command.params = params
        return self.complete(self.client.call_async(request)).result

    def wait_for_mode(self, state):
        self.wait(lambda: self.states[-1].current_state == state)
        self.wait(lambda: self.results and self.results[-1].result == ModeResult.SUCCEEDED)
        self.results.clear()

    def standing(self):
        self.streaming = True
        result = self.command()
        self.assertEqual(result.result, ModeResult.ACCEPTED)
        self.wait_for_mode(ModeState.STANDING)

    def test_state_and_legacy_mode_transitions(self):
        initial = self.states[-1]
        self.assertEqual((initial.current_state, initial.target_state),
                         (ModeState.PASSIVE, ModeState.PASSIVE))
        self.assertEqual((initial.cur_algorithm, initial.cur_action, initial.cur_params),
                         (-1, -1, -1))
        self.assertFalse(initial.fault)
        self.assertEqual(initial.fault_str, '')

        self.streaming = True
        for name, data, expected in (
            ('robot_action', 1, ModeState.STANDING),
            ('robot_mode', 1, ModeState.STANDING),
            ('robot_action', 2, ModeState.PASSIVE),
        ):
            client = self.node.create_client(RobotCmd, name)
            self.wait(client.service_is_ready)
            response = self.complete(client.call_async(RobotCmd.Request(data=data)))
            self.assertEqual(response.success, 1)
            self.wait_for_mode(expected)
            if name == 'robot_mode':
                velocity = Twist()
                velocity.linear.y = -.12
                self.run_for(.15, lambda: self.vel_pub.publish(velocity))
                self.wait(lambda: self.states[-1].current_state == ModeState.LOCOMOTION)
        self.assertTrue(all(c.params == 0 for c in self.commands))

    def test_unsupported_commands_are_not_forwarded(self):
        for code in (1, 2, 3, 6, 255):
            result = self.command(code, target=255)
            self.assertEqual(result.result, ModeResult.REJECTED)
        invalid_mode = self.command(target=5)
        self.assertEqual(invalid_mode.result, ModeResult.REJECTED)
        self.run_for(.1)
        self.assertFalse(self.commands)

    def test_invalid_action_params_are_not_forwarded(self):
        for target, params in ((1, 0), (2, 1), (255, 0), (0, 4), (0, 255)):
            result = self.command(ModeCommand.START_ACTION, target, params)
            self.assertEqual(result.result, ModeResult.REJECTED)
            self.assertEqual(result.reason, 'INVALID_ACTION_OR_PARAMS')
        self.run_for(.05)
        self.assertFalse(self.commands)

    def prepare_action(self):
        self.standing()
        self.follow_body_height = True
        self.telemetry.joints.position = [0., -.3, .6, 0., .3, -.6,
                                         0., .3, -.6, 0., -.3, .6]
        self.run_for(.08)
        self.results.clear()

    def test_action_schemes_state_and_completion(self):
        result = self.command(ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS,
                              ModeCommand.KNEE_BACK)
        self.assertEqual(result.result, ModeResult.REJECTED)
        self.assertIn('STANDING', result.reason)
        self.prepare_action()
        for params in (ModeCommand.KNEE_INSIDE, ModeCommand.KNEE_BACK,
                       ModeCommand.KNEE_FRONT, ModeCommand.KNEE_OUTSIDE):
            self.results.clear()
            count = len(self.commands)
            response = self.command(ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS, params)
            self.assertEqual(response.result, ModeResult.ACCEPTED)
            self.wait(lambda: len(self.commands) == count + 1)
            wire = self.commands[-1]
            self.assertEqual((wire.command, wire.target, wire.params),
                             (ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS, params))
            if params != ModeCommand.KNEE_INSIDE:
                self.wait(lambda: self.states[-1].current_state == ModeState.ACTION)
                self.assertEqual((self.states[-1].cur_action, self.states[-1].cur_params),
                                 (ModeCommand.CHANGE_KINEMATICS, params))
                self.assertEqual(self.states[-1].target_state, ModeState.STANDING)
                for command, target in ((ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS),
                                        (ModeCommand.SET_MODE, ModeCommand.PASSIVE)):
                    busy = self.command(command, target)
                    self.assertEqual((busy.result, busy.reason), (ModeResult.BUSY, 'BRIDGE_BUSY'))
                self.assertEqual(len(self.commands), count + 1)
            self.wait(lambda: any(r.result == ModeResult.SUCCEEDED for r in self.results), timeout=18.)
            self.wait(lambda: self.states[-1].current_state == ModeState.STANDING)
            self.assertEqual((self.states[-1].cur_action, self.states[-1].cur_params), (-1, -1))
            self.assertFalse(any(r.result in (ModeResult.FAILED, ModeResult.UNKNOWN) for r in self.results))
        self.assertEqual(self.command().result, ModeResult.ACCEPTED)

    def test_estop_interrupts_action(self):
        self.prepare_action()
        result = self.command(ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS,
                              ModeCommand.KNEE_OUTSIDE)
        self.assertEqual(result.result, ModeResult.ACCEPTED)
        self.wait(lambda: self.states[-1].current_state == ModeState.ACTION)
        self.assertEqual(self.command(ModeCommand.ESTOP).result, ModeResult.SUCCEEDED)
        self.wait(lambda: self.states[-1].current_state == ModeState.FAILURE)
        self.wait(lambda: any(r.result == ModeResult.FAILED for r in self.results))
        failed = next(r for r in self.results if r.result == ModeResult.FAILED)
        self.assertIn('ESTOP', failed.reason)

    def test_disconnect_during_action_does_not_replay(self):
        self.prepare_action()
        result = self.command(ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS,
                              ModeCommand.KNEE_BACK)
        self.assertEqual(result.result, ModeResult.ACCEPTED)
        self.wait(lambda: self.states[-1].current_state == ModeState.ACTION)
        count = len(self.commands)
        self.fsm.terminate()
        self.assertEqual(self.fsm.wait(timeout=3), 0)
        self.wait(lambda: any(r.result == ModeResult.UNKNOWN for r in self.results))
        self.assertEqual(self.results[-1].reason, 'FSM_UNAVAILABLE')
        self.assertEqual(self.command(ModeCommand.START_ACTION, ModeCommand.CHANGE_KINEMATICS,
                                      ModeCommand.KNEE_BACK).result, ModeResult.UNKNOWN)
        self.assertEqual(len(self.commands), count)

    def test_estop_interrupts_pending_mode_and_clear_fault_leaves_passive(self):
        self.assertEqual(self.command().result, ModeResult.ACCEPTED)
        self.wait(lambda: self.states[-1].target_state == ModeState.STANDING)
        self.assertEqual(self.states[-1].current_state, ModeState.PASSIVE)
        stop = self.command(ModeCommand.ESTOP)
        self.assertEqual(stop.result, ModeResult.SUCCEEDED)
        self.wait(lambda: self.states[-1].current_state == ModeState.FAILURE)
        self.wait(lambda: any(r.result == ModeResult.FAILED for r in self.results))
        failed = next(r for r in self.results if r.result == ModeResult.FAILED)
        self.assertIn('ESTOP', failed.reason)
        self.assertTrue(self.states[-1].fault)
        self.assertIn('ESTOP', self.states[-1].fault_str)
        self.assertEqual(self.command(ModeCommand.CLEAR_FAULT).result, ModeResult.REJECTED)

        self.streaming = True
        self.run_for(.3)
        self.assertEqual(self.command(ModeCommand.CLEAR_FAULT).result, ModeResult.SUCCEEDED)
        self.wait(lambda: self.states[-1].current_state == ModeState.PASSIVE)
        self.assertEqual(self.states[-1].target_state, ModeState.PASSIVE)
        self.assertFalse(self.states[-1].fault)
        self.assertEqual(self.states[-1].fault_str, '')

    def test_streams_drive_locomotion_and_expire(self):
        self.standing()
        result = self.command(target=3)
        self.assertIn(result.result, (ModeResult.ACCEPTED, ModeResult.SUCCEEDED))
        self.wait_for_mode(ModeState.STANDING)
        velocity = Twist(); velocity.linear.x = .12; velocity.angular.z = .2
        self.run_for(.3, lambda: self.vel_pub.publish(velocity))
        self.wait(lambda: self.states[-1].current_state == ModeState.LOCOMOTION)
        pose = Twist(); pose.linear.z = .21
        self.run_for(.8, lambda: self.pose_pub.publish(pose))
        self.assertEqual(self.packets[-1].cmd_vel, (0.,) * 6)
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .21, places=5)
        self.wait(lambda: self.states[-1].current_state == ModeState.STANDING)
        velocity.linear.x = .18
        self.run_for(.15, lambda: self.vel_pub.publish(velocity))
        self.assertAlmostEqual(self.packets[-1].cmd_vel[0], .18, places=5)
        self.assertAlmostEqual(self.packets[-1].cmd_pose[2], .21, places=5)
        self.wait(lambda: self.states[-1].current_state == ModeState.LOCOMOTION)
        self.run_for(.1)  # Drain the final input before checking the absence of periodic output.
        count = len(self.packets)
        gait = GaitParams(t_sw=.2, t_st=.3, stride_height=.04,
                          gait_offsets=[0., .5, .5, 0.], standing=False)
        bad = Twist(); bad.linear.x = float('nan')
        self.run_for(.9, lambda: (self.gait_pub.publish(gait), self.pose_pub.publish(bad),
                                 self.vel_pub.publish(bad)))
        self.assertEqual(len(self.packets), count)
        self.wait(lambda: self.states[-1].current_state == ModeState.STANDING)
        self.assertTrue(self.gait_packets)
        self.assertAlmostEqual(self.gait_packets[-1].t_sw, .2, places=5)
        self.assertEqual(self.gait_packets[-1].gait_type, (0., .5, .5, 0.))

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

    def test_disconnect_reports_unknown_without_replaying_mode(self):
        self.assertEqual(self.command().result, ModeResult.ACCEPTED)
        self.fsm.terminate()
        self.assertEqual(self.fsm.wait(timeout=3), 0)
        self.wait(lambda: any(r.result == ModeResult.UNKNOWN for r in self.results))
        self.assertEqual(self.results[-1].reason, 'FSM_UNAVAILABLE')
        self.assertEqual(self.command().result, ModeResult.UNKNOWN)
        self.assertEqual(len(self.commands), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
