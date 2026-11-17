import os
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

try:
    import lcm
except ModuleNotFoundError:
    print("Python LCM bindings are unavailable")
    sys.exit(77)


class NodeTest:
    def __init__(self, url, node):
        self.bus = lcm.LCM(url)
        self.node = node
        self.states = deque(maxlen=1000)
        self.results = deque()
        self.servo_cmds = 0
        self.bus.subscribe("ROBOT_MODE_STATE", lambda _, data: self.states.append(State.decode(data)))
        self.bus.subscribe("ROBOT_MODE_RESULT", lambda _, data: self.results.append(Result.decode(data)))
        self.bus.subscribe("SERVO_CMD", self.receive_servo_cmd)

        self.streaming = False
        self.last_publish = 0.0
        self.robot_state = RobotState()
        self.robot_state.body.orientation_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.robot_state.legs.contact_states = [True] * 4
        self.robot_state.readiness = [True] * 18 + [False]

    def receive_servo_cmd(self, _channel, _data):
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

    def effective(self, current, target):
        self.wait_for(
            lambda: self.states and
            (self.states[-1].current_state, self.states[-1].target_state) == (current, target),
            f"state {current} targeting {target}",
        )

    def velocity(self, x=0.0, y=0.0, yaw=0.0):
        msg = RobotCommand()
        msg.cmd_vel = [x, y, 0.0, 0.0, 0.0, yaw]
        msg.active_legs = [True] * 4
        self.bus.publish("ROBOT_CMD", msg.encode())

    def send(self, command, target=0):
        msg = Command()
        msg.command = command
        msg.target = target
        self.bus.publish("ROBOT_MODE_CMD", msg.encode())

    def result(self, command, expected, reason=None):
        self.wait_for(lambda: self.results, f"result {expected} for command {command}")
        result = self.results.popleft()
        assert (result.command, result.result) == (command, expected), (
            result.command, result.result, result.reason, command, expected
        )
        if expected in (Result.REJECTED, Result.CANCELED, Result.FAILED):
            assert result.reason
        if reason:
            assert reason in result.reason, result.reason

    def mode(self, target, expected):
        self.send(Command.SET_MODE, target)
        self.result(Command.SET_MODE, Result.ACCEPTED)
        self.result(Command.SET_MODE, Result.SUCCEEDED)
        self.state(expected)

    def clear_fault(self):
        self.send(Command.CLEAR_FAULT)
        self.result(Command.CLEAR_FAULT, Result.SUCCEEDED)
        state = self.state(State.PASSIVE)
        assert state.target_state == State.PASSIVE
        self.remains_in(State.PASSIVE, 0.23)


def exercise(test):
    test.state(State.PASSIVE)
    test.remains_in(State.PASSIVE, 0.06)
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
    test.result(Command.SET_MODE, Result.SUCCEEDED)
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
    test.mode(Command.STANDING, State.STANDING)

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

    test.mode(Command.STANDING, State.STANDING)
    telemetry.readiness[17] = False
    test.state(State.FAILURE, "ROBOT_STATE_NOT_READY")
    telemetry.readiness[17] = True
    test.run_for(0.24)
    test.clear_fault()

    test.mode(Command.STANDING, State.STANDING)
    telemetry.joints.position[10] = -2.61
    test.state(State.FAILURE, "JOINT_LIMIT: hip_joint_L2")
    test.run_for(0.24)
    test.send(Command.CLEAR_FAULT)
    test.result(Command.CLEAR_FAULT, Result.REJECTED, "hip_joint_L2")
    telemetry.joints.position[10] = 0.0
    test.run_for(0.03)
    test.clear_fault()

    test.mode(Command.STANDING, State.STANDING)
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
    test.run_for(0.24)
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
    estimate(0.0, 0.0)
    test.effective(State.LOCOMOTION, State.PASSIVE)
    test.result(Command.SET_MODE, Result.SUCCEEDED)
    test.state(State.PASSIVE)
    assert any(state.current_state == State.LAYDOWN for state in test.states)


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

    with tempfile.TemporaryFile() as log, subprocess.Popen(
        [executable], env=env, stdout=log, stderr=subprocess.STDOUT
    ) as process:
        try:
            test = NodeTest(url, process)
            run_test(test)
            process.send_signal(signal.SIGINT if servo_url else signal.SIGTERM)
            assert process.wait(timeout=2) == 0, "Node failed to stop"
            while select.select([test.bus.fileno()], [], [], 0.04)[0]:
                test.bus.handle()
            assert test.servo_cmds == 0, "Node published SERVO_CMD"
        except BaseException:
            if process.poll() is None:
                process.kill()
                process.wait()
            log.seek(0)
            print(log.read().decode(errors="replace"))
            raise


if __name__ == "__main__":
    executable, generator, schema_dir = sys.argv[1:]
    with tempfile.TemporaryDirectory() as bindings:
        subprocess.run(
            [generator, "--python", "--ppath", bindings,
             *map(str, sorted(Path(schema_dir).glob("*.lcm")))],
            check=True,
        )
        sys.path.insert(0, bindings)
        from mors_msgs import gait_params_msg as GaitParams
        from mors_msgs import robot_cmd_msg as RobotCommand
        from mors_msgs import robot_mode_cmd_msg as Command
        from mors_msgs import robot_mode_result_msg as Result
        from mors_msgs import robot_mode_state_msg as State
        from mors_msgs import robot_state_msg as RobotState

        run_node(executable, exercise, servo_url=False)
        run_node(executable, lambda test: test.state(State.PASSIVE), servo_url=True)
