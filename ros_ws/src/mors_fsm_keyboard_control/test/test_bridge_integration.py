"""Drive the actual terminal node and bridge on isolated ROS/LCM transports."""

import os
from pathlib import Path
import pty
import signal
import socket
import subprocess
import sys
import termios
import time

from ament_index_python.packages import get_package_prefix
import lcm
import pytest
import yaml

from ros2lcm_bridge_runtime.lcm_types.mors_msgs import (
    gait_params_msg, robot_cmd_msg, robot_mode_cmd_msg, robot_mode_result_msg,
    robot_mode_state_msg,
)


def test_terminal_control_through_bridge(tmp_path):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
        port.bind(('127.0.0.1', 0))
        number = port.getsockname()[1]
    url = f'udpm://239.255.76.67:{number}?ttl=0'
    config = tmp_path / 'config'
    config.mkdir()
    channels = dict(robot_cmd='KEYBOARD_CMD', gait_params='KEYBOARD_GAIT',
                    robot_mode_cmd='KEYBOARD_MODE_CMD', robot_mode_state='KEYBOARD_STATE',
                    robot_mode_result='KEYBOARD_RESULT')
    (config / 'channels.yaml').write_text(yaml.safe_dump(channels))
    env = {**os.environ, 'CONFIGPATH': str(config), 'LCM_LOCOMOTION_URL': url,
           'ROS_DOMAIN_ID': '174', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    namespace = f'/keyboard_test_{number}'
    bus = lcm.LCM(url)
    state = robot_mode_state_msg()
    state.current_state = state.target_state = state.PASSIVE
    state.cur_algorithm = state.cur_action = state.cur_params = -1
    commands, packets, gaits = [], [], []

    def command_received(_, data):
        command = robot_mode_cmd_msg.decode(data)
        commands.append(command)
        if command.command == command.SET_MODE:
            # Model FSM permission semantics: allowing walking can finish in STANDING.
            state.current_state = state.target_state = (
                state.PASSIVE if command.target == command.IDLE else state.STANDING)
        else:
            assert command.command == command.START_ACTION
            assert command.target == command.CHANGE_KINEMATICS
            assert state.current_state == state.STANDING
        for code in (robot_mode_result_msg.ACCEPTED, robot_mode_result_msg.SUCCEEDED):
            result = robot_mode_result_msg()
            result.command, result.result, result.reason = command.command, code, ''
            bus.publish(channels['robot_mode_result'], result.encode())

    bus.subscribe(channels['robot_mode_cmd'], command_received)
    bus.subscribe(channels['robot_cmd'], lambda _, data: packets.append(
        (time.monotonic(), robot_cmd_msg.decode(data))))
    bus.subscribe(channels['gait_params'], lambda _, data: gaits.append(
        gait_params_msg.decode(data)))
    bridge_exe = Path(get_package_prefix('ros2lcm_bridge')) / 'lib/ros2lcm_bridge/bridge_node'
    master, slave = pty.openpty()
    settings = termios.tcgetattr(slave)
    processes = []
    logs = []

    def start(args, stdin=None):
        log = (tmp_path / f'process-{len(processes)}.log').open('w+')
        process = subprocess.Popen(args, stdin=stdin, stdout=log, stderr=log, env=env)
        processes.append(process)
        logs.append(log)
        return process

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            bus.publish(channels['robot_mode_state'], state.encode())
            for _ in range(128):
                if not bus.handle_timeout(0):
                    break
            for process, log in zip(processes, logs):
                if process.poll() is not None:
                    log.seek(0)
                    pytest.fail(log.read())
            time.sleep(.005)

    def wait(predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                for log in logs:
                    log.seek(0)
                pytest.fail('Timeout\n' + '\n'.join(log.read() for log in logs))
            pump(.02)

    def keyboard_output():
        logs[-1].seek(0)
        return logs[-1].read()

    try:
        start([str(bridge_exe), '--ros-args', '-r', f'__ns:={namespace}',
               '-p', 'legacy_services:=false'])
        keyboard_process = start([
            sys.executable, '-m', 'mors_fsm_keyboard_control.mors_fsm_keyboard_control',
            '--ros-args', '-r', f'__ns:={namespace}', '-p', 'use_sim_time:=true'], stdin=slave)
        wait(lambda: packets and 'FSM state: PASSIVE' in keyboard_output())
        os.write(master, b'\n')
        wait(lambda: len(commands) == 1 and 'command: SUCCEEDED' in keyboard_output())
        assert commands[-1].target == robot_mode_cmd_msg.STANDING

        os.write(master, b' ')
        wait(lambda: len(commands) == 2 and 'control mode: LOCOMOTION' in keyboard_output())
        assert commands[-1].target == robot_mode_cmd_msg.LOCOMOTION
        # Hold a constant command beyond both the keyboard deadman and FSM timeout.
        start_time = time.monotonic()
        for _ in range(10):
            os.write(master, b'w')
            pump(.1)
        moving = [(stamp, packet) for stamp, packet in packets if stamp > start_time + .1]
        assert len(moving) >= 30
        assert all(packet.cmd_vel[0] == pytest.approx(.3) for _, packet in moving)
        assert max(b[0] - a[0] for a, b in zip(moving, moving[1:])) < .15
        pump(.55)
        assert packets[-1][1].cmd_vel == (0.,) * 6

        os.write(master, b'9\x1b[A\x1b[1;5A\x1b[1;2Aqa')
        pump(.2)
        assert packets[-1][1].cmd_vel[1] == pytest.approx(.54)
        assert packets[-1][1].cmd_vel[5] == pytest.approx(1.8)
        assert gaits[-1].t_sw == pytest.approx(.19)
        assert gaits[-1].stride_height == pytest.approx(.045)
        assert gaits[-1].t_st == pytest.approx(.2)
        pump(.4)
        assert packets[-1][1].cmd_pose[2] == pytest.approx(.208, abs=1e-5)

        os.write(master, b' ')
        wait(lambda: len(commands) == 3)
        pump(.2)
        assert commands[-1].target == robot_mode_cmd_msg.STANDING
        assert packets[-1][1].cmd_vel == (0.,) * 6

        for scheme in range(4):
            os.write(master, b'!@#$'[scheme:scheme + 1])
            wait(lambda: len(commands) == 4 + scheme)
            pump(.35)
            assert (commands[-1].command, commands[-1].target, commands[-1].params) == (
                robot_mode_cmd_msg.START_ACTION, robot_mode_cmd_msg.CHANGE_KINEMATICS, scheme)
            assert packets[-1][1].cmd_vel == (0.,) * 6
            assert gaits[-1].t_st == pytest.approx(.2)  # Shift+digits did not change speed.

        os.write(master, b'\n')
        wait(lambda: len(commands) == 8 and 'FSM state: PASSIVE' in keyboard_output())
        assert commands[-1].target == robot_mode_cmd_msg.IDLE
        keyboard_process.send_signal(signal.SIGINT)
        keyboard_process.wait(timeout=3.)
        assert keyboard_process.returncode == 0
        assert termios.tcgetattr(slave) == settings
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.)
        for log in logs:
            log.close()
        os.close(master)
        os.close(slave)
