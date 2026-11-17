"""Single-client LCM protocol. Commands are sent once; replies have no IDs."""
from dataclasses import dataclass
import math
import time

from mors_ros_msgs.msg import ModeResult
from rclpy.task import Future

from .lcm_types.mors_msgs import robot_mode_cmd_msg as WireCommand
from .lcm_types.mors_msgs import robot_mode_result_msg as WireResult


RESULT_CODES = {
    WireResult.ACCEPTED: ModeResult.ACCEPTED,
    WireResult.SUCCEEDED: ModeResult.SUCCEEDED,
    WireResult.REJECTED: ModeResult.REJECTED,
    WireResult.BUSY: ModeResult.BUSY,
    WireResult.CANCELED: ModeResult.CANCELED,
    WireResult.TIMEOUT: ModeResult.TIMEOUT,
    WireResult.FAILED: ModeResult.FAILED,
}


def finite_vector(values):
    return all(math.isfinite(v) and abs(v) <= 3.4028234663852886e38 for v in values)


@dataclass
class Outcome:
    result: int
    reason: str = ''


@dataclass
class Pending:
    started: float
    first: Future


class Protocol:
    def __init__(self, publish, emit, *, clock=time.monotonic,
                 ack_timeout=2., result_timeout=30., state_timeout=1.):
        self.publish, self.emit = publish, emit
        self.clock = clock
        self.ack_timeout, self.result_timeout = ack_timeout, result_timeout
        self.state_timeout = state_timeout
        self.state_received = -math.inf
        self.pending = {}
        self.uncertain = False

    def fresh(self):
        return self.clock() - self.state_received <= self.state_timeout

    def on_state(self):
        self.state_received = self.clock()

    def command(self, command, target=0):
        first = Future()
        mode_targets = (WireCommand.IDLE, WireCommand.STANDING, WireCommand.LOCOMOTION)
        if (command not in (WireCommand.SET_MODE, WireCommand.ESTOP, WireCommand.CLEAR_FAULT)
                or (command == WireCommand.SET_MODE and target not in mode_targets)):
            return self._resolve(first, Outcome(ModeResult.REJECTED, 'INVALID_COMMAND_OR_MODE'))
        if not self.fresh():
            return self._resolve(first, Outcome(ModeResult.UNKNOWN, 'FSM_UNAVAILABLE'))
        if self.uncertain and command != WireCommand.ESTOP:
            return self._resolve(first, Outcome(
                ModeResult.UNKNOWN, 'RESULT_UNKNOWN: restart bridge before new commands'))
        # One regular operation; ESTOP has a distinct reply code and can interrupt it.
        if self.pending and (command != WireCommand.ESTOP or command in self.pending):
            return self._resolve(first, Outcome(ModeResult.BUSY, 'BRIDGE_BUSY'))

        wire = WireCommand()
        wire.command = command
        wire.target = target if command == WireCommand.SET_MODE else 0
        wire.params = 0
        self.publish(wire)
        if self.uncertain:
            # ESTOP remains available, but its uncorrelated reply cannot restore certainty.
            return self._resolve(first, Outcome(ModeResult.UNKNOWN, 'RESULT_UNKNOWN'))
        self.pending[command] = Pending(self.clock(), first)
        return first

    def _resolve(self, first, outcome):
        if not first.done():
            first.set_result(outcome)
        self.emit(outcome)
        return first

    def _unknown_all(self, reason):
        self.uncertain = True
        for request in self.pending.values():
            self._resolve(request.first, Outcome(ModeResult.UNKNOWN, reason))
        self.pending.clear()

    def on_result(self, result):
        code = RESULT_CODES.get(result.result)
        request = self.pending.get(result.command)
        if code is None or request is None:
            return
        if code != ModeResult.ACCEPTED:
            del self.pending[result.command]
        elif request.first.done():
            return
        self._resolve(request.first, Outcome(code, result.reason))

    def tick(self):
        now = self.clock()
        for request in self.pending.values():
            reason = None
            if not self.fresh():
                reason = 'FSM_UNAVAILABLE'
            elif now - request.started >= self.result_timeout:
                reason = 'RESULT_UNKNOWN'
            elif not request.first.done() and now - request.started >= self.ack_timeout:
                reason = 'ACK_UNKNOWN'
            if reason:
                self._unknown_all(reason)
                break
