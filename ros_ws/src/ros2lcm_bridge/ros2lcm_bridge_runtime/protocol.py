"""Single-client LCM protocol. Commands are sent once; replies have no IDs."""
from dataclasses import dataclass, field
import math
import time

from rclpy.task import Future


def finite_vector(values):
    return all(math.isfinite(v) and abs(v) <= 3.4028234663852886e38 for v in values)


@dataclass
class Outcome:
    result: int
    reason: str = ''


@dataclass
class Pending:
    wire: object
    started: float
    first: Future = field(default_factory=Future)
    final: Future = field(default_factory=Future)


class Protocol:
    def __init__(self, command_type, publish, emit, *, clock=time.monotonic,
                 ack_timeout=2., result_timeout=30., state_timeout=1.):
        self.command_type, self.publish, self.emit = command_type, publish, emit
        self.clock = clock
        self.ack_timeout, self.result_timeout = ack_timeout, result_timeout
        self.state_timeout = state_timeout
        self.state = None
        self.state_received = -math.inf
        self.pending = []
        self.uncertain = False

    def fresh(self):
        return self.state is not None and self.clock() - self.state_received <= self.state_timeout

    def on_state(self, state):
        self.state, self.state_received = state, self.clock()
        return True

    def command(self, command, target=0):
        return self._submit(command, target, service_command=True)

    def start_action(self, action_id):
        return self._submit(self.command_type.START_ACTION, action_id)

    def cancel_action(self):
        return self._submit(self.command_type.CANCEL_ACTION)

    def _submit(self, command, target=0, service_command=False):
        wire = self.command_type()
        wire.command = command
        wire.target = target if command in (wire.SET_MODE, wire.SET_ALGORITHM, wire.START_ACTION) else 0
        request = Pending(wire, self.clock())
        allowed = ((wire.SET_MODE, wire.SET_ALGORITHM, wire.ESTOP, wire.CLEAR_FAULT, wire.SET_EXTERNAL)
                   if service_command else range(7))
        if command not in allowed or (command == wire.SET_MODE and target not in (0, 2, 3)):
            return self._finish(request, Outcome(2, 'INVALID_COMMAND_OR_MODE'))
        if command == wire.SET_ALGORITHM and not 0 <= target <= 255:
            return self._finish(request, Outcome(2, 'INVALID_ALGORITHM_ID'))
        if command == wire.START_ACTION and not 0 <= target <= 255:
            return self._finish(request, Outcome(2, 'INVALID_ACTION_ID'))
        if not self.fresh():
            return self._finish(request, Outcome(255, 'FSM_UNAVAILABLE'))
        if self.uncertain and command != wire.ESTOP:
            return self._finish(request, Outcome(255, 'RESULT_UNKNOWN: restart bridge before new commands'))
        # One regular operation; cancellation and ESTOP have distinct reply codes.
        interrupt = command in (wire.CANCEL_ACTION, wire.ESTOP)
        if self.pending and (not interrupt or any(p.wire.command == command for p in self.pending)):
            return self._finish(request, Outcome(3, 'BRIDGE_BUSY'))
        if command == wire.CANCEL_ACTION and not any(
                p.wire.command == wire.START_ACTION for p in self.pending):
            return self._finish(request, Outcome(2, 'No active action'))
        if self.uncertain:
            # ESTOP remains available, but its uncorrelated reply cannot restore certainty.
            self.publish(wire)
            return self._finish(request, Outcome(255, 'RESULT_UNKNOWN'))
        self.pending.append(request)
        self.publish(wire)
        return request

    def _finish(self, request, outcome):
        if not request.first.done():
            request.first.set_result(outcome)
        if not request.final.done():
            request.final.set_result(outcome)
        if request in self.pending:
            self.pending.remove(request)
        self.emit(outcome)
        return request

    def _unknown_all(self, reason):
        for request in list(self.pending):
            self._finish(request, Outcome(255, reason))
        self.uncertain = True

    def on_result(self, result):
        if result.result not in range(8) or self.uncertain:
            return
        outcome = Outcome(result.result, result.reason)
        request = next((p for p in self.pending if p.wire.command == result.command), None)
        if request is None:
            return
        if result.result == 0:
            if not request.first.done():
                request.first.set_result(outcome)
                self.emit(outcome)
        else:
            self._finish(request, outcome)

    def tick(self):
        now = self.clock()
        for request in self.pending:
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
