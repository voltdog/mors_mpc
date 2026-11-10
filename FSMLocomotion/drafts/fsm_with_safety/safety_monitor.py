"""Programmable, latched safety checks for a quadruped robot.

This is supervisory SOFTWARE monitoring, not a certified safety controller.
Independent hardware/drive protection and a watchdog are still required.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from numbers import Real
from typing import Mapping


@dataclass(frozen=True)
class JointLimits:
    minimum_rad: float
    maximum_rad: float

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.minimum_rad)
            or not math.isfinite(self.maximum_rad)
            or self.minimum_rad >= self.maximum_rad
        ):
            raise ValueError("Invalid joint limits")


@dataclass(frozen=True)
class JointFeedback:
    positions_rad: Mapping[str, float]
    stamp_monotonic_s: float  # Host-side receive timestamp from time.monotonic()


@dataclass(frozen=True)
class SafetyFault:
    code: str
    description: str
    timestamp_monotonic_s: float


class SafetyMonitor:
    """Checks encoder feedback watchdog and measured joint-angle limits.

    First detected error is latched until explicitly reset after a safe reading.
    Keep this separate from state transition guards and drive firmware limits.
    """

    def __init__(
        self,
        joint_limits: Mapping[str, JointLimits],
        *,
        max_feedback_age_s: float = 0.1,
    ) -> None:
        if not joint_limits:
            raise ValueError("Configure limits for every controlled joint")
        if not math.isfinite(max_feedback_age_s) or max_feedback_age_s <= 0:
            raise ValueError("max_feedback_age_s must be positive")
        self.joint_limits = dict(joint_limits)
        self.max_feedback_age_s = max_feedback_age_s
        self._latched_fault: SafetyFault | None = None

    @property
    def fault(self) -> SafetyFault | None:
        return self._latched_fault

    def evaluate(
        self,
        feedback: JointFeedback | None,
        *,
        feedback_required: bool = True,
        fsm_state: str | None = None,
    ) -> SafetyFault | None:
        """Stateless check; joint limits are inactive in IDLE and STANDUP."""
        now = time.monotonic()

        def fault(code: str, description: str) -> SafetyFault:
            return SafetyFault(code, description, now)

        if not feedback_required:
            return None

        if feedback is None:
            return fault("JOINT_FEEDBACK_MISSING", "No joint feedback")

        stamp = feedback.stamp_monotonic_s
        if not isinstance(stamp, Real) or not math.isfinite(stamp):
            return fault("JOINT_FEEDBACK_INVALID", "Non-finite feedback timestamp")
        age = now - stamp
        if age < -0.001:
            return fault("JOINT_FEEDBACK_INVALID", "Feedback timestamp is in the future")
        if age > self.max_feedback_age_s:
            return fault("JOINT_FEEDBACK_STALE", f"Joint feedback age {age:.3f} s")

        # Encoder watchdog still checks the availability of all joint readings.
        check_angle_limits = fsm_state not in ("IDLE", "STANDUP")
        for joint, limits in self.joint_limits.items():
            if joint not in feedback.positions_rad:
                return fault("JOINT_MISSING", f"Missing encoder position for {joint}")
            q = feedback.positions_rad[joint]
            if not isinstance(q, Real) or not math.isfinite(q):
                return fault("JOINT_INVALID", f"{joint}: non-finite angle: {q}")
            if check_angle_limits and not (limits.minimum_rad <= q <= limits.maximum_rad):
                return fault(
                    "JOINT_LIMIT",
                    f"{joint}: q={q:.4f} rad outside "
                    f"[{limits.minimum_rad:.4f}, {limits.maximum_rad:.4f}] rad",
                )
        return None

    def check(
        self,
        feedback: JointFeedback | None,
        *,
        feedback_required: bool = True,
        fsm_state: str | None = None,
    ) -> SafetyFault | None:
        """Evaluate and latch the first error, returning the latched fault."""
        detected = self.evaluate(
            feedback, feedback_required=feedback_required, fsm_state=fsm_state
        )
        if self._latched_fault is None and detected is not None:
            self._latched_fault = detected
        return self._latched_fault

    def is_encoder_feedback_valid(self, feedback: JointFeedback | None) -> bool:
        """Актуальность и корректность энкодеров без проверки углов в IDLE."""
        return (self._latched_fault is None
                and self.evaluate(feedback, fsm_state="IDLE") is None)

    def clear_fault(
        self,
        feedback: JointFeedback | None,
        *,
        operator_acknowledged: bool,
        external_estop_released: bool,
    ) -> bool:
        """Manual reset only when external stop released and feedback is safe."""
        if not operator_acknowledged or not external_estop_released:
            return False
        # Сброс переводит FSM в IDLE; в IDLE лимиты углов не проверяются.
        if self.evaluate(feedback, fsm_state="IDLE") is not None:
            return False
        self._latched_fault = None
        return True
