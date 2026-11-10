"""Начальная диагностика перед выходом FSM из IDLE.

Только проверка готовности; этот модуль не выдаёт ESTOP.
Проверку актуальности энкодеров переиспользует из SafetyMonitor.
"""

from __future__ import annotations

import math
import time
from numbers import Real
from typing import Protocol

from safety_monitor import JointFeedback, SafetyMonitor


class ReadinessStatus(Protocol):
    """Нужные поля RobotStatus без циклического импорта из модуля FSM."""

    drives_ready: bool
    contact_sensors_ready: bool
    estimator_ready: bool
    joint_feedback: JointFeedback | None
    drive_status_stamp_s: float | None
    contact_stamp_s: float | None
    estimator_stamp_s: float | None
    contacts: tuple[bool, bool, bool, bool]
    vx: float
    vy: float
    yaw_rate: float


class ReadinessChecker:
    def __init__(
        self,
        safety_monitor: SafetyMonitor,
        *,
        max_status_age_s: float = 0.15,
        stable_time_s: float = 0.20,
    ) -> None:
        if not math.isfinite(max_status_age_s) or max_status_age_s <= 0:
            raise ValueError("max_status_age_s must be positive")
        if not math.isfinite(stable_time_s) or stable_time_s < 0:
            raise ValueError("stable_time_s must be non-negative")
        self.safety_monitor = safety_monitor
        self.max_status_age_s = max_status_age_s
        self.stable_time_s = stable_time_s
        self.stable_since: float | None = None
        self.last_errors: tuple[str, ...] = ()

    def reset(self) -> None:
        self.stable_since = None
        self.last_errors = ()

    def _fresh(self, stamp: float | None, now: float) -> bool:
        if not isinstance(stamp, Real) or not math.isfinite(stamp):
            return False
        age = now - stamp
        return -0.001 <= age <= self.max_status_age_s

    def is_ready(self, status: ReadinessStatus) -> bool:
        """True, если все источники корректны непрерывно stable_time_s."""
        now = time.monotonic()
        errors: list[str] = []

        if not status.drives_ready:
            errors.append("Приводы не готовы")
        if not self._fresh(status.drive_status_stamp_s, now):
            errors.append("Нет актуального статуса приводов")
        if not self.safety_monitor.is_encoder_feedback_valid(status.joint_feedback):
            errors.append("Нет корректных актуальных данных энкодеров")

        if not status.contact_sensors_ready:
            errors.append("Датчики контакта не готовы")
        if not self._fresh(status.contact_stamp_s, now):
            errors.append("Нет актуальных данных датчиков контакта")
        if len(status.contacts) != 4 or any(type(c) is not bool for c in status.contacts):
            errors.append("Некорректные данные датчиков контакта")

        if not status.estimator_ready:
            errors.append("Оценщик состояния не готов")
        if not self._fresh(status.estimator_stamp_s, now):
            errors.append("Нет актуальных данных оценщика состояния")
        if any(not isinstance(v, Real) or not math.isfinite(v)
               for v in (status.vx, status.vy, status.yaw_rate)):
            errors.append("Некорректные скорости оценщика состояния")

        self.last_errors = tuple(errors)
        if errors:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= self.stable_time_s
