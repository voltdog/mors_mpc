import math
import os
import re
import select
import signal
import socket
import shutil
import subprocess
import sys
import tempfile
import time
from collections import deque
from contextlib import ExitStack
from pathlib import Path

try:
    import lcm
except ModuleNotFoundError:
    print("Python LCM bindings are unavailable")
    sys.exit(77)


STANDUP_TIMEOUT = 8.0
STANDUP_POSE = (
    0.0, -1.57, 2.9, 0.0, 1.57, -2.9,
    0.0, 1.57, -2.9, 0.0, -1.57, 2.9,
)


class NodeTest:
    def __init__(self, url, node):
        self.bus = lcm.LCM(url)
        self.node = node
        self.states = deque(maxlen=1000)
        self.results = deque()
        self.non_damping_cmds = 0
        self.last_servo_damped = False
        self.servo_cmds = 0
        self.servo_trace = None
        self.follow_action_joints = False
        self.follow_body_height = False
        self.bus.subscribe("ROBOT_MODE_STATE", lambda _, data: self.states.append(State.decode(data)))
        self.bus.subscribe("ROBOT_MODE_RESULT", lambda _, data: self.results.append(Result.decode(data)))
        self.bus.subscribe("SERVO_CMD", self.receive_servo_cmd)
        self.bus.subscribe("MPC_CMD", self.receive_body_reference)

        self.streaming = False
        self.last_publish = 0.0
        self.robot_state = RobotState()
        self.robot_state.body.orientation_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.robot_state.body.position[2] = 0.20
        self.robot_state.joints.position = list(STANDUP_POSE)
        self.robot_state.legs.r1_pos = [0.18, -0.13, 0.0]
        self.robot_state.legs.l1_pos = [0.18, 0.13, 0.0]
        self.robot_state.legs.r2_pos = [-0.18, -0.13, 0.0]
        self.robot_state.legs.l2_pos = [-0.18, 0.13, 0.0]
        self.robot_state.legs.contact_states = [True] * 4
        self.robot_state.readiness = [True] * 18 + [False]

    def receive_body_reference(self, _channel, data):
        if self.follow_body_height:
            cmd = MpcCommand.decode(data)
            self.robot_state.body.position[2] = cmd.cmd_pose[2]
            self.robot_state.body.lin_vel[2] = cmd.cmd_vel[2]

    def receive_servo_cmd(self, _channel, data):
        cmd = ServoCommand.decode(data)
        if self.follow_action_joints and all(math.isclose(kp, 12.0, abs_tol=1e-6) for kp in cmd.kp):
            self.robot_state.joints.position = list(cmd.position)
            self.robot_state.joints.velocity = list(cmd.velocity)
        if self.servo_trace is not None:
            self.servo_trace.append((time.monotonic(), cmd))
        self.last_servo_damped = (
            not any(any(getattr(cmd, field)) for field in ("position", "velocity", "torque", "kp"))
            and all(math.isclose(kd, 0.4, rel_tol=0.0, abs_tol=1e-6) for kd in cmd.kd)
        )
        if not self.last_servo_damped:
            self.non_damping_cmds += 1
        self.servo_cmds += 1

    def pump(self):
        assert self.node.poll() is None, f"Node exited early with status {self.node.returncode}"
        now = time.monotonic()
        if self.streaming and now - self.last_publish >= 0.005:
            # Оставляем время источника нулевым, проверяется локальное время приёма.
            self.bus.publish("ROBOT_STATE", self.robot_state.encode())
            self.last_publish = now
        if select.select([self.bus.fileno()], [], [], 0.002)[0]:
            self.bus.handle()

    def wait_for(self, predicate, description, timeout=2.0):
        end = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < end, f"Timed out waiting for {description}"
            self.pump()

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump()

    def state(self, expected, fault=None):
        self.wait_for(
            lambda: self.states and self.states[-1].current_state == expected,
            f"state {expected}",
        )
        state = self.states[-1]
        assert state.fault == (fault is not None), state.fault_str
        if fault:
            assert fault in state.fault_str, state.fault_str
            assert state.target_state == State.FAILURE
        return state

    def remains_in(self, expected, seconds):
        self.states.clear()
        self.run_for(seconds)
        assert self.states, "No mode state messages"
        assert all(state.current_state == expected for state in self.states)

    def remains_damped(self, state, seconds):
        self.wait_for(lambda: self.last_servo_damped, "damping SERVO_CMD")
        before, non_damping = self.servo_cmds, self.non_damping_cmds
        self.remains_in(state, seconds)
        assert self.servo_cmds > before + 10
        assert self.non_damping_cmds == non_damping

    def effective(self, current, target):
        self.wait_for(
            lambda: self.states and
            (self.states[-1].current_state, self.states[-1].target_state) == (current, target),
            f"state {current} targeting {target}",
        )

    def velocity(self, x=0.0, y=0.0, yaw=0.0):
        msg = RobotCommand()
        msg.cmd_pose[2] = 0.20
        msg.cmd_vel = [x, y, 0.0, 0.0, 0.0, yaw]
        msg.active_legs = [True] * 4
        self.bus.publish("ROBOT_CMD", msg.encode())

    def send(self, command, target=0, params=0):
        msg = Command()
        msg.command = command
        msg.target = target
        msg.params = params
        self.bus.publish("ROBOT_MODE_CMD", msg.encode())

    def result(self, command, expected, reason=None, *, timeout=4.0):
        self.wait_for(lambda: self.results, f"result {expected} for command {command}", timeout=timeout)
        result = self.results.popleft()
        assert (result.command, result.result) == (command, expected), (
            result.command, result.result, result.reason, command, expected,
            (self.states[-1].current_state, self.states[-1].fault_str) if self.states else None,
        )
        if expected in (Result.REJECTED, Result.CANCELED, Result.FAILED):
            assert result.reason
        if reason:
            assert reason in result.reason, result.reason

    def mode(self, target, expected, *, timeout=4.0):
        self.send(Command.SET_MODE, target)
        self.result(Command.SET_MODE, Result.ACCEPTED)
        self.result(Command.SET_MODE, Result.SUCCEEDED, timeout=timeout)
        self.state(expected)

    def clear_fault(self):
        self.send(Command.CLEAR_FAULT)
        self.result(Command.CLEAR_FAULT, Result.SUCCEEDED)
        state = self.state(State.PASSIVE)
        assert state.target_state == State.PASSIVE
        self.remains_damped(State.PASSIVE, 0.23)


def exercise(test):
    test.state(State.PASSIVE)
    test.remains_damped(State.PASSIVE, 0.06)
    for command, target, reason in (
        (99, 0, "Unknown command"),
        (Command.SET_EXTERNAL, 0, "Unsupported command"),
        (Command.SET_MODE, Command.EXTERNAL, "Unsupported mode target"),
        (Command.CLEAR_FAULT, 0, "No fault to clear"),
    ):
        test.send(command, target)
        test.result(command, Result.REJECTED, reason)

    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.PASSIVE, 0.07)
    assert test.states[-1].target_state == State.STANDING
    for target in (Command.LOCOMOTION, Command.STANDING):
        test.send(Command.SET_MODE, target)
        test.result(Command.SET_MODE, Result.CANCELED)
        test.result(Command.SET_MODE, Result.ACCEPTED)

    telemetry = test.robot_state
    test.streaming = True
    telemetry.readiness[0] = False
    telemetry.joints.position[0] = 1.4
    test.remains_in(State.PASSIVE, 0.24)
    telemetry.readiness[0] = True
    test.state(State.STANDUP)
    telemetry.joints.position[0] = 0.0
    test.result(Command.SET_MODE, Result.SUCCEEDED, timeout=STANDUP_TIMEOUT)
    test.state(State.STANDING)
    telemetry.readiness[18] = True
    test.remains_in(State.STANDING, 0.05)
    telemetry.readiness[18] = False
    test.remains_in(State.STANDING, 0.05)

    test.velocity(x=-0.2)
    test.remains_in(State.STANDING, 0.57)
    test.mode(Command.LOCOMOTION, State.STANDING)
    test.effective(State.STANDING, State.STANDING)
    test.remains_in(State.STANDING, 0.06)

    gait = GaitParams()
    gait.t_st, gait.t_sw, gait.stride_height = 0.3, 0.2, 0.06
    gait.gait_type = [0.0, 0.5, 0.5, 0.0]
    test.bus.publish("GAIT_PARAMS", gait.encode())
    test.velocity(x=-0.2)
    test.state(State.LOCOMOTION)

    # Вертикальная скорость в мире даёт продольную скорость наклонённого корпуса.
    telemetry.body.orientation_quaternion = [0.0, 0.3826834, 0.0, 0.9238795]
    telemetry.body.lin_vel = [0.0, 0.0, 0.1]
    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.34)
    telemetry.body.lin_vel = [0.0] * 3
    telemetry.legs.contact_states[3] = False
    test.remains_in(State.LOCOMOTION, 0.34)
    telemetry.legs.contact_states[3] = True
    test.velocity(x=-0.2)
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.state(State.STANDING)
    test.remains_in(State.STANDING, 0.06)
    telemetry.body.orientation_quaternion = [0.0, 0.0, 0.0, 1.0]

    test.velocity()
    test.run_for(0.04)
    test.mode(Command.LOCOMOTION, State.STANDING)
    test.velocity(yaw=-0.2)
    test.state(State.LOCOMOTION)
    telemetry.legs.contact_states[3] = False
    test.velocity()
    test.effective(State.LOCOMOTION, State.STANDING)
    test.velocity(y=-0.2)
    test.effective(State.LOCOMOTION, State.LOCOMOTION)
    telemetry.legs.contact_states[3] = True
    test.effective(State.LOCOMOTION, State.STANDING)  # Свежая боковая команда истекла.
    test.state(State.STANDING)
    test.velocity(y=0.2)
    test.state(State.LOCOMOTION)  # Разрешение сохраняется после автоостановки.
    test.mode(Command.STANDING, State.STANDING)
    recovery_transitions(test)
    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)

    test.streaming = False
    test.state(State.FAILURE, "ROBOT_STATE_STALE")
    test.send(Command.CLEAR_FAULT)
    test.result(Command.CLEAR_FAULT, Result.REJECTED, "ROBOT_STATE age")
    telemetry.readiness[17] = False
    test.streaming = True
    test.run_for(0.24)
    test.send(Command.CLEAR_FAULT)
    test.result(Command.CLEAR_FAULT, Result.REJECTED, "readiness bit 17")
    telemetry.readiness[17] = True
    test.run_for(0.24)
    test.clear_fault()

    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    telemetry.readiness[17] = False
    test.state(State.FAILURE, "ROBOT_STATE_NOT_READY")
    telemetry.readiness[17] = True
    test.run_for(0.24)
    test.clear_fault()

    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    telemetry.joints.position[10] = -2.61
    test.state(State.FAILURE, "JOINT_LIMIT: hip_joint_L2")
    test.run_for(0.24)
    test.send(Command.CLEAR_FAULT)
    test.result(Command.CLEAR_FAULT, Result.REJECTED, "hip_joint_L2")
    telemetry.joints.position[10] = -1.57
    test.run_for(0.03)
    test.clear_fault()

    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    telemetry.body.position[0] = 0.14
    telemetry.body.lin_vel[0] = 0.25
    test.state(State.LOCOMOTION)
    test.velocity(x=0.2)
    test.send(Command.SET_MODE, Command.LOCOMOTION)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.08)
    assert not test.results
    test.send(Command.ESTOP)
    test.send(Command.SET_MODE, Command.IDLE)
    test.result(Command.SET_MODE, Result.FAILED, "ESTOP")
    test.result(Command.ESTOP, Result.SUCCEEDED)
    test.result(Command.SET_MODE, Result.REJECTED, "FAILURE")
    test.state(State.FAILURE, "ESTOP")
    telemetry.body.position[0] = telemetry.body.lin_vel[0] = 0.0
    test.remains_damped(State.FAILURE, 0.24)
    test.clear_fault()
    test.velocity(x=0.2)
    test.remains_in(State.PASSIVE, 0.08)
    assert not test.results, "Unhandled mode results"


def recovery_transitions(test):
    body = test.robot_state.body

    def estimate(x, vx):
        body.position[0] = x
        body.lin_vel[0] = vx

    estimate(0.14, 0.0)
    test.remains_in(State.STANDING, 0.07)
    estimate(0.0, 0.25)
    test.remains_in(State.STANDING, 0.07)
    estimate(0.14, 0.25)
    test.state(State.LOCOMOTION)

    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    estimate(0.055, 0.02)
    test.remains_in(State.LOCOMOTION, 0.10)
    assert test.states[-1].target_state == State.LOCOMOTION and not test.results
    test.robot_state.legs.contact_states[0] = False
    estimate(0.02, 0.02)
    test.remains_in(State.LOCOMOTION, 0.10)
    assert test.states[-1].target_state == State.LOCOMOTION and not test.results, "Recovery exited before its window filled"
    test.effective(State.LOCOMOTION, State.STANDING)
    estimate(0.09, 0.02)
    test.effective(State.LOCOMOTION, State.LOCOMOTION)
    assert not test.results, "STANDING was confirmed while still in RECOVERY"
    test.robot_state.legs.contact_states[0] = True
    estimate(0.02, 0.02)
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.state(State.STANDING)

    estimate(0.16, 0.25)
    test.state(State.LOCOMOTION)
    test.send(Command.SET_MODE, Command.LOCOMOTION)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.07)
    assert not test.results, "RECOVERY was mistaken for WALKING"
    test.velocity(y=0.2)
    estimate(0.075, 0.095)  # Проходит пороги WALKING, но не STANDING.
    deadline = time.monotonic() + 4.0
    while not test.results:
        assert time.monotonic() < deadline, "Timed out waiting for recovery to finish"
        test.velocity(y=0.2)
        test.run_for(0.05)
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.effective(State.LOCOMOTION, State.LOCOMOTION)
    estimate(0.0, 0.0)
    test.mode(Command.STANDING, State.STANDING)

    estimate(0.14, 0.25)
    test.state(State.LOCOMOTION)
    test.send(Command.SET_MODE, Command.IDLE)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    estimate(0.08, 0.02)
    test.remains_in(State.LOCOMOTION, 0.34)
    assert test.states[-1].target_state == State.LOCOMOTION and not test.results
    initial = [0.12, -0.8, 1.9, -0.13, 0.7, -1.8, 0.14, 0.8, -1.7, -0.16, -0.7, 1.6]
    folded = [0.12, -1.57, 2.9, -0.13, 1.57, -2.9, 0.14, 1.57, -2.9, -0.16, -1.57, 2.9]
    test.robot_state.joints.position = initial
    test.servo_trace = []
    estimate(0.0, 0.0)
    test.effective(State.LOCOMOTION, State.PASSIVE)
    check_laydown(test, initial, folded)
    test.robot_state.joints.position = list(STANDUP_POSE)


def check_laydown(test, initial, folded):
    test.state(State.LAYDOWN)
    assert not test.results, "IDLE was confirmed before laydown finished"
    test.result(Command.SET_MODE, Result.SUCCEEDED, timeout=5.0)
    test.state(State.PASSIVE)
    trace = test.servo_trace
    test.servo_trace = None
    test.remains_damped(State.PASSIVE, 0.06)

    def matches(actual, expected):
        return all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-5)
                   for a, b in zip(actual, expected))

    def gains(cmd, kp, kd):
        return matches(cmd.kp, [kp] * 12) and matches(cmd.kd, [kd] * 12)

    def find_sample(description, predicate, start=0):
        for i in range(start, len(trace)):
            if predicate(trace[i][1]):
                return i
        raise AssertionError(f"Missing laydown command for {description}")

    centered = folded.copy()
    centered[0::3] = [0.0] * 4
    start = find_sample("first position command", lambda cmd: gains(cmd, 12.0, 0.4))
    assert matches(trace[start][1].position, initial)
    fold = find_sample("folded pose", lambda cmd: matches(cmd.position, folded), start + 1)
    center = find_sample("centered pose", lambda cmd: matches(cmd.position, centered), fold + 1)
    damp = find_sample("increased damping", lambda cmd: gains(cmd, 0.0, 0.8), center + 1)
    release = find_sample("zero gains", lambda cmd: gains(cmd, 0.0, 0.0), damp + 1)

    for _, cmd in trace[start:fold + 1]:
        assert matches(cmd.position[0::3], initial[0::3])
    for _, cmd in trace[fold:center + 1]:
        assert matches(cmd.position[1::3], folded[1::3])
        assert matches(cmd.position[2::3], folded[2::3])
    for _, cmd in trace[start:damp]:
        assert gains(cmd, 12.0, 0.4)
    for _, cmd in trace[damp:release]:
        assert gains(cmd, 0.0, 0.8) and matches(cmd.position, folded)
    for _, cmd in trace[start:release + 1]:
        assert not any(cmd.velocity) and not any(cmd.torque)

    for first, last, expected in ((start, fold, 2.0), (fold, center, 0.5), (damp, release, 0.5)):
        actual = trace[last][0] - trace[first][0]
        assert math.isclose(actual, expected, abs_tol=0.1), (actual, expected)


def exercise_stopping(test):
    test.state(State.PASSIVE)
    test.streaming = True
    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    test.mode(Command.LOCOMOTION, State.STANDING)

    # При нулевой измеренной скорости всё равно нужны 50 снимков после входа.
    test.velocity(x=0.06)
    test.state(State.LOCOMOTION)
    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.45)
    assert not test.results, "STANDING was confirmed before the speed window filled"
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.state(State.STANDING)

    test.mode(Command.LOCOMOTION, State.STANDING)
    test.velocity(x=0.2)
    test.state(State.LOCOMOTION)
    body = test.robot_state.body
    body.lin_vel[0] = 0.8
    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.60)
    assert not test.results, "STANDING was confirmed at high measured speed"

    body.lin_vel[0] = 0.0
    test.robot_state.legs.contact_states[0] = False
    test.remains_in(State.LOCOMOTION, 0.60)
    assert not test.results, "STANDING was confirmed without four contacts"
    test.robot_state.legs.contact_states[0] = True
    test.remains_in(State.LOCOMOTION, 0.12)
    body.lin_vel[0] = 0.2
    test.remains_in(State.LOCOMOTION, 0.10)
    body.lin_vel[0] = 0.0
    test.remains_in(State.LOCOMOTION, 0.20)
    assert not test.results, "Acceleration did not reset the stability hold"
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.state(State.STANDING)

    test.mode(Command.LOCOMOTION, State.STANDING)
    test.velocity(x=1.0)
    test.state(State.LOCOMOTION)
    body.lin_vel[0] = 1.0
    test.send(Command.SET_MODE, Command.STANDING)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.remains_in(State.LOCOMOTION, 0.10)
    test.send(Command.ESTOP)
    test.result(Command.SET_MODE, Result.FAILED, "ESTOP")
    test.result(Command.ESTOP, Result.SUCCEEDED)
    test.state(State.FAILURE, "ESTOP")
    test.remains_damped(State.FAILURE, 0.10)


def exercise_laydown(test):
    test.state(State.PASSIVE)
    test.streaming = True
    # M переопределяет знак колена L1. В нестандартной позе сохраняются знаки и нули.
    for initial, folded in (
        ([0.12, -0.8, 1.9, -0.13, 0.7, 0.1, 0.14, -0.8, 1.7, -0.16, 0.7, -1.6],
         [0.12, -1.57, 2.9, -0.13, 1.57, -2.9, 0.14, -1.57, 2.9, -0.16, 1.57, -2.9]),
        ([0.12, 0.8, 1.9, -0.13, 0.0, -1.8, 0.14, -0.8, 0.0, -0.16, 0.7, 1.6],
         [0.12, 1.57, 2.9, -0.13, 0.0, -2.9, 0.14, -1.57, 0.0, -0.16, 1.57, 2.9]),
    ):
        test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
        test.robot_state.joints.position = initial
        test.run_for(0.05)
        test.servo_trace = []
        test.send(Command.SET_MODE, Command.IDLE)
        test.result(Command.SET_MODE, Result.ACCEPTED)
        check_laydown(test, initial, folded)
        test.robot_state.joints.position = list(STANDUP_POSE)

    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    test.send(Command.SET_MODE, Command.IDLE)
    test.result(Command.SET_MODE, Result.ACCEPTED)
    test.state(State.LAYDOWN)
    test.remains_in(State.LAYDOWN, 0.12)
    test.send(Command.ESTOP)
    test.result(Command.SET_MODE, Result.FAILED, "ESTOP")
    test.result(Command.ESTOP, Result.SUCCEEDED)
    test.state(State.FAILURE, "ESTOP")
    test.remains_damped(State.FAILURE, 0.24)


def exercise_switch_kin_sch(test):
    test.state(State.PASSIVE)
    test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, Command.KNEE_BACK)
    test.result(Command.START_ACTION, Result.REJECTED, "STANDING")
    test.streaming = True
    test.mode(Command.STANDING, State.STANDING, timeout=STANDUP_TIMEOUT)
    test.follow_body_height = True
    initial = [0.06, -0.55, 1.10, -0.07, 0.65, -1.20,
               0.08, 0.75, -1.30, -0.09, -0.85, 1.40]
    # Independent encoder revolutions must not alter the chosen knee direction.
    turns = [2 * math.pi, -2 * math.pi, 4 * math.pi, -4 * math.pi]
    for leg, turn in enumerate(turns):
        initial[3 * leg + 2] += turn
    test.robot_state.joints.position = initial.copy()
    test.run_for(0.08)
    for target, params, reason in ((99, 0, "target"), (Command.CHANGE_KINEMATICS, 99, "parameter")):
        test.send(Command.START_ACTION, target, params)
        test.result(Command.START_ACTION, Result.REJECTED, reason)
    test.robot_state.joints.position[2] = 0.0
    test.run_for(0.04)
    test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, Command.KNEE_FRONT)
    test.result(Command.START_ACTION, Result.REJECTED, "zero")
    test.robot_state.joints.position = initial.copy()
    test.run_for(0.04)

    def normalized(angle):
        value = math.remainder(angle, 2 * math.pi)
        return value - 2 * math.pi if value >= math.pi else value

    def angles(cmd):
        result = list(cmd.position)
        result[2::3] = map(normalized, result[2::3])
        return result

    def matches(actual, expected):
        return all(math.isclose(a, b, abs_tol=3e-5, rel_tol=0.0) for a, b in zip(actual, expected))

    # Already reached goal must leave WBIC commands untouched.
    test.servo_trace = []
    test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, Command.KNEE_INSIDE)
    test.result(Command.START_ACTION, Result.ACCEPTED)
    test.result(Command.START_ACTION, Result.SUCCEEDED)
    test.state(State.STANDING)
    assert all(all(kp < 3.0 for kp in cmd.kp) for _, cmd in test.servo_trace), "No-op took joint control"
    test.follow_action_joints = True

    for params, signs in ((Command.KNEE_BACK, [1, -1, 1, -1]),
                          (Command.KNEE_FRONT, [-1, 1, -1, 1]),
                          (Command.KNEE_OUTSIDE, [-1, 1, 1, -1]),
                          (Command.KNEE_INSIDE, [1, -1, -1, 1])):
        before = test.robot_state.joints.position.copy()
        before[2::3] = map(normalized, before[2::3])
        target = before.copy()
        for leg, sign in enumerate(signs):
            j = 3 * leg + 2
            if (before[j] > 0.0) != (sign > 0):
                target[j - 1], target[j] = -before[j - 1], -before[j]
        test.servo_trace = []
        test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, params)
        test.result(Command.START_ACTION, Result.ACCEPTED)
        current = test.state(State.ACTION)
        assert current.cur_action == Command.CHANGE_KINEMATICS and current.cur_params == params
        assert current.target_state == State.STANDING
        test.send(Command.SET_MODE, Command.IDLE)
        test.result(Command.SET_MODE, Result.REJECTED, "Action")
        test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, params)
        test.result(Command.START_ACTION, Result.BUSY)
        test.send(Command.CANCEL_ACTION)
        test.result(Command.CANCEL_ACTION, Result.REJECTED, "Unsupported")
        test.result(Command.START_ACTION, Result.SUCCEEDED, timeout=18.0)
        current = test.state(State.STANDING)
        assert current.cur_action == current.cur_params == -1
        trace = test.servo_trace
        test.servo_trace = None
        full = [i for i, (_, cmd) in enumerate(trace)
                if all(math.isclose(kp, 12.0, abs_tol=1e-6) for kp in cmd.kp)]
        test.pump()
        assert full, "Missing joint control commands"
        assert any(matches(angles(cmd), target) and not any(cmd.velocity)
                   for _, cmd in trace[full[0]:full[-1] + 1]), "Final measured-angle target missing"
        test.pump()
        # Old WBIC results must never replace direct commands during the action.
        assert all(all(math.isclose(kp, 12.0, abs_tol=1e-6) for kp in cmd.kp)
                   for _, cmd in trace[full[0]:full[-1] + 1])
        test.pump()
        assert any(2.1 < cmd.kp[0] < 11.9 for _, cmd in trace[:full[0]]), "Missing entry gain ramp"
        assert any(2.1 < cmd.kp[0] < 11.9 for _, cmd in trace[full[-1] + 1:]), "Missing return gain ramp"
        assert all(abs(v) <= 1.0 + 2e-5 for _, cmd in trace for v in cmd.velocity), "Speed limit exceeded"
        test.pump()
        front_moves = not matches(before[:6], target[:6])
        rear_moves = not matches(before[6:], target[6:])
        if front_moves and rear_moves:
            rear_start = next(i for i in full if any(abs(v) > 1e-5 for v in trace[i][1].velocity[6:]))
            assert any(matches(angles(cmd)[:6], target[:6]) and not any(cmd.velocity)
                       for _, cmd in trace[full[0]:rear_start]), "Rear pair started before front finished"
        for i in full:
            if i % 100 == 0:
                test.pump()
            q = angles(trace[i][1])
            assert matches(q[0::3], before[0::3]), "Abad changed"
            if not front_moves:
                assert matches(q[:6], before[:6]), "Skipped front pair moved"
            if not rear_moves:
                assert matches(q[6:], before[6:]), "Skipped rear pair moved"
        test.run_for(0.04)

    # ESTOP interrupts an accepted action and inhibits subsequent outputs.
    test.send(Command.START_ACTION, Command.CHANGE_KINEMATICS, Command.KNEE_OUTSIDE)
    test.result(Command.START_ACTION, Result.ACCEPTED)
    test.state(State.ACTION)
    test.run_for(0.48)
    test.send(Command.ESTOP)
    test.result(Command.START_ACTION, Result.FAILED, "ESTOP")
    test.result(Command.ESTOP, Result.SUCCEEDED)
    test.remains_damped(State.FAILURE, 0.10)
    assert not test.results


def run_node(executable, run_test, *, servo_url):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port_socket:
        port_socket.bind(("", 0))
        port = port_socket.getsockname()[1]
    url = f"udpm://239.255.76.67:{port}?ttl=0"
    env = os.environ.copy()
    for key in ("CONFIGPATH", "LCM_LOCOMOTION_URL", "LCM_DEFAULT_URL", "LCM_SERVO_URL"):
        env.pop(key, None)
    if servo_url:
        env["LCM_LOCOMOTION_URL"] = env["LCM_SERVO_URL"] = url
    else:
        env["LCM_DEFAULT_URL"] = url

    with ExitStack() as stack:
        if run_test in (exercise, exercise_switch_kin_sch):
            config = Path(stack.enter_context(tempfile.TemporaryDirectory()))
            shutil.copytree(Path(__file__).resolve().parents[2] / "config", config, dirs_exist_ok=True)
            robot = config / "robot.yaml"
            settings = robot.read_text()
            if run_test == exercise:
                # Фиксируем пороги сценария независимо от пользовательской настройки робота.
                for key, thresholds in (
                    ("recovery_entry", "{position_m: 0.10, velocity_mps: 0.20}"),
                    ("recovery_exit_walking", "{position_m: 0.07, velocity_mps: 0.12}"),
                    ("recovery_exit_standing", "{position_m: 0.04, velocity_mps: 0.08}"),
                ):
                    settings = re.sub(rf"(?m)^(\s*){key}:.*$", rf"\1{key}: {thresholds}", settings)
            else:
                settings = re.sub(r"(?m)^(\s*)debug_mode:.*$", r"\1debug_mode: true", settings)
            robot.write_text(settings)
            env["CONFIGPATH"] = str(config)
        log = stack.enter_context(tempfile.TemporaryFile())
        process = stack.enter_context(subprocess.Popen(
            [executable], env=env, stdout=log, stderr=subprocess.STDOUT))
        try:
            test = NodeTest(url, process)
            run_test(test)
            before = test.servo_cmds
            process.send_signal(signal.SIGINT if servo_url else signal.SIGTERM)
            assert process.wait(timeout=2) == 0, "Node failed to stop"
            while select.select([test.bus.fileno()], [], [], 0.04)[0]:
                test.bus.handle()
            assert test.servo_cmds > before and test.last_servo_damped, "Missing final damping SERVO_CMD"
        except BaseException:
            if process.poll() is None:
                process.kill()
                process.wait()
            log.seek(0)
            print(log.read().decode(errors="replace"))
            raise


if __name__ == "__main__":
    executable, generator, schema_dir = sys.argv[1:4]
    with tempfile.TemporaryDirectory() as bindings:
        subprocess.run(
            [generator, "--python", "--ppath", bindings,
             *map(str, sorted(Path(schema_dir).glob("*.lcm")))],
            check=True,
        )
        sys.path.insert(0, bindings)
        from mors_msgs import gait_params_msg as GaitParams
        from mors_msgs import robot_cmd_msg as RobotCommand
        from mors_msgs import mpc_cmd_msg as MpcCommand
        from mors_msgs import robot_mode_cmd_msg as Command
        from mors_msgs import robot_mode_result_msg as Result
        from mors_msgs import robot_mode_state_msg as State
        from mors_msgs import servo_cmd_msg as ServoCommand
        from mors_msgs import robot_state_msg as RobotState

        if sys.argv[4:] == ["switch_kin_sch"]:
            run_node(executable, exercise_switch_kin_sch, servo_url=True)
        else:
            run_node(executable, exercise, servo_url=False)
            run_node(executable, exercise_laydown, servo_url=True)
            run_node(executable, exercise_stopping, servo_url=False)
