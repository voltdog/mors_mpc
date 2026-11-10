"""Пример FSM робота с BFS, guards и независимым SafetyMonitor.

Python 3.10+. Потоков нет: RobotFSM.update() вызывается из главного цикла.
LocomotionControl здесь — демонстрационная заглушка; подключите реальный контроллер.
"""

from __future__ import annotations

import math
import time
from abc import ABC
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto

from safety_monitor import JointFeedback, JointLimits, SafetyMonitor
from readiness_checker import ReadinessChecker


class StateID(Enum):
    IDLE = auto()
    STANDUP = auto()
    STANDING = auto()
    LOCOMOTION = auto()
    LAYDOWN = auto()
    ACTION = auto()
    EXTERNAL = auto()
    FAILURE = auto()


class Event(Enum):
    START_STANDUP = auto()
    START_STANDING = auto()
    START_LOCOMOTION = auto()
    START_LAYDOWN = auto()
    START_IDLE = auto()
    START_ACTION = auto()
    START_EXTERNAL = auto()
    ESTOP = auto()
    CLEAR_FAULT = auto()


@dataclass
class RobotStatus:
    # Условия выхода из IDLE
    drives_ready: bool = False
    contact_sensors_ready: bool = False
    estimator_ready: bool = False

    # Условия выхода из LOCOMOTION
    vx: float = 0.0                      # м/с, в системе координат корпуса
    vy: float = 0.0                      # м/с
    yaw_rate: float = 0.0                # рад/с
    contacts: tuple[bool, bool, bool, bool] = (False, False, False, False)
    gait_stopped: bool = False           # Планировщик походки закончил шаги

    # Аварийный сигнал (от внешней системы безопасности)
    estop_active: bool = False  # ВНЕШНИЙ ESTOP (кнопка/другой контроллер)
    drives_enabled: bool = False  # Реальное состояние силовых выходов
    joint_feedback: JointFeedback | None = None

    # Отдельные отметки времени ПОЛУЧЕНИЯ данных от каждого источника.
    # Обновлять только при поступлении новых пакетов, не в каждом update() FSM.
    drive_status_stamp_s: float | None = None
    contact_stamp_s: float | None = None
    estimator_stamp_s: float | None = None


class LocomotionControl:
    """Заглушка: замените реальной реализацией LocomotionControl."""

    def __init__(self):
        self.velocity_command = (0.0, 0.0, 0.0)
        self.outputs_inhibited = False

    def set_data_input_mode(self, input_mode):
        # Заглушка: в реальном роботе переключает между режимами CONTROLLER, WBC, JOINTS 
        self.input_mode = input_mode
    
    def set_robot_cmd(self, robot_cmd):
        pass  # Заглушка: в реальном роботе передаём команду в контроллер движения.

    def set_wbc_cmd(self, wbc_cmd):
        pass  # Заглушка: в реальном роботе передаём команду в WBC-контроллер.

    def set_joints_cmd(self, joints_cmd):
        pass  # Заглушка: в реальном роботе передаём команду в приводы.

    def set_locomotion_mode(self, mode: str) -> None:
        # Заглушка: в реальном роботе переключает между режимами STAND и WALK
        # Может быть вызвана только если self.input_mode == "CONTROLLER" 
        pass

    def start_thread(self) -> None:
        # Заглушка: в реальном роботе запускает поток управления движением.
        pass

    def stop_thread(self) -> None:
        # Заглушка: в реальном роботе останавливает поток управления движением.
        pass

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> None: # в будущем убрать
        if self.outputs_inhibited:
            return
        self.velocity_command = (vx, vy, yaw_rate)

    def step(self) -> None: # в будущем убрать
        if self.outputs_inhibited:
            return
        # Здесь вызывается ваш алгоритм управления движением.
        # RobotStatus обновляется сенсорами/оценщиком, а не этой заглушкой.
        pass

    def emergency_stop(self) -> None:
        # Демонстрация. На реальном роботе независимая защита обязательна.
        already_inhibited = self.outputs_inhibited
        self.outputs_inhibited = True
        self.velocity_command = (0.0, 0.0, 0.0)
        # ПРОДАКШЕН: заблокировать выходные команды и инициировать аварийное
        # действие в драйверах/контроллере безопасности. Не только cmd_vel=0!
        if not already_inhibited:
            print("Controller: outputs inhibited, emergency stop requested")

    def enable_outputs(self) -> None:
        # Заглушка: на реальном роботе нужен подтверждённый re-enable приводов.
        self.outputs_inhibited = False


class State(ABC):
    def __init__(self, fsm: RobotFSM):
        self.fsm = fsm

    def on_enter(self) -> None:
        print(f"Enter {type(self).__name__}")

    def on_update(self) -> Event | None:
        return None

    def can_exit(self, next_state: StateID) -> bool:
        """Проверка перед on_exit(); по умолчанию переход разрешён."""
        return True

    def on_exit(self) -> None:
        print(f"Exit  {type(self).__name__}")

    def on_abort(self) -> None:
        # Аварийный выход: НЕ отправлять команды управления.
        print(f"Abort {type(self).__name__}")


class IdleState(State):
    def on_enter(self) -> None:
        super().on_enter()
        self.fsm.readiness_checker.reset()

    def on_update(self) -> Event | None:
        # Фоновая диагностика даже без запроса перехода.
        self.fsm.readiness_checker.is_ready(self.fsm.status)
        return None

    def can_exit(self, next_state: StateID) -> bool:
        return (
            next_state == StateID.STANDUP
            and not self.fsm.controller.outputs_inhibited
            and self.fsm.readiness_checker.is_ready(self.fsm.status)
        )


class StandupState(State):
    def on_enter(self) -> None:
        super().on_enter()
        self.ticks = 0

    def on_update(self) -> Event | None:
        # Заглушка: в реальном роботе — проверка завершения вставания.
        self.ticks += 1
        if self.ticks >= 3:
            return Event.START_STANDING
        return None


class StandingState(State):
    pass


class LocomotionState(State):
    LINEAR_VEL_THRESHOLD = 0.05     # м/с
    ANGULAR_VEL_THRESHOLD = 0.10    # рад/с
    STABLE_TIME = 0.30             # с

    def __init__(self, fsm: RobotFSM):
        super().__init__(fsm)
        self.stable_since: float | None = None
        self.stop_requested = False

    def on_enter(self) -> None:
        super().on_enter()
        self.stable_since = None
        self.stop_requested = False

    def on_update(self) -> Event | None:
        # Любой запрос другого устойчивого состояния требует сначала
        # остановиться и перейти в STANDING.
        target = self.fsm.target_state
        self.stop_requested = (
            target is not None and target != StateID.LOCOMOTION
        )

        if self.stop_requested:
            self.fsm.controller.set_velocity(0.0, 0.0, 0.0)
        else:
            # Отменён запрос остановки — больше не накапливаем время
            # выполнения условий выхода.
            self.stable_since = None

        # Контроллер должен продолжать работать до завершения остановки.
        self.fsm.controller.step()
        return None

    def can_exit(self, next_state: StateID) -> bool:
        if next_state != StateID.STANDING or not self.stop_requested:
            self.stable_since = None
            return False

        s = self.fsm.status
        linear_speed = math.hypot(s.vx, s.vy)
        velocity_ok = (
            math.isfinite(linear_speed)
            and math.isfinite(s.yaw_rate)
            and linear_speed < self.LINEAR_VEL_THRESHOLD
            and abs(s.yaw_rate) < self.ANGULAR_VEL_THRESHOLD
        )
        contacts_ok = (
            s.contact_sensors_ready
            and len(s.contacts) == 4
            and all(s.contacts)
            and s.gait_stopped
        )

        if not (velocity_ok and contacts_ok):
            self.stable_since = None
            return False

        now = time.monotonic()
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= self.STABLE_TIME

    def on_exit(self) -> None:
        self.stable_since = None
        self.stop_requested = False
        super().on_exit()


class LaydownState(State):
    def on_enter(self) -> None:
        super().on_enter()
        self.ticks = 0

    def on_update(self) -> Event | None:
        # Заглушка: в реальном роботе — проверка завершения укладывания.
        self.ticks += 1
        if self.ticks >= 3:
            return Event.START_IDLE
        return None


class ActionState(State):
    pass


class ExternalState(State):
    pass


class FailureState(State):
    def on_enter(self) -> None:
        super().on_enter()
        # Идемпотентная повторная защита на случай внешнего вызова перехода.
        self.fsm.controller.emergency_stop()
        print("FAILURE: движение запрещено")


class RobotFSM:
    """Цель -> BFS -> следующий переход -> can_exit -> on_exit/on_enter."""

    EDGES = {
        (StateID.IDLE, StateID.STANDUP): Event.START_STANDUP,
        (StateID.STANDUP, StateID.STANDING): Event.START_STANDING,
        (StateID.STANDING, StateID.LAYDOWN): Event.START_LAYDOWN,
        (StateID.LAYDOWN, StateID.IDLE): Event.START_IDLE,
        (StateID.STANDING, StateID.LOCOMOTION): Event.START_LOCOMOTION,
        (StateID.LOCOMOTION, StateID.STANDING): Event.START_STANDING,
        (StateID.STANDING, StateID.ACTION): Event.START_ACTION,
        (StateID.ACTION, StateID.STANDING): Event.START_STANDING,
        (StateID.STANDING, StateID.EXTERNAL): Event.START_EXTERNAL,
        (StateID.EXTERNAL, StateID.STANDING): Event.START_STANDING,
    }
    TRANSIENT = {StateID.STANDUP, StateID.LAYDOWN}
    TARGETS = {
        StateID.IDLE,
        StateID.STANDING,
        StateID.LOCOMOTION,
        StateID.ACTION,
        StateID.EXTERNAL,
    }

    def __init__(
        self,
        status: RobotStatus | None = None,
        controller: LocomotionControl | None = None,
        *,
        safety_monitor: SafetyMonitor,
        readiness_checker: ReadinessChecker | None = None,
    ) -> None:
        self.status = status if status is not None else RobotStatus()
        self.controller = controller if controller is not None else LocomotionControl()
        self.safety_monitor = safety_monitor
        self.readiness_checker = (readiness_checker or ReadinessChecker(safety_monitor))
        self.target_state: StateID | None = None
        self._last_blocked: tuple[StateID, StateID] | None = None

        # Один экземпляр каждого состояния на всё время работы.
        self.states: dict[StateID, State] = {
            StateID.IDLE: IdleState(self),
            StateID.STANDUP: StandupState(self),
            StateID.STANDING: StandingState(self),
            StateID.LOCOMOTION: LocomotionState(self),
            StateID.LAYDOWN: LaydownState(self),
            StateID.ACTION: ActionState(self),
            StateID.EXTERNAL: ExternalState(self),
            StateID.FAILURE: FailureState(self),
        }
        self.state_id = StateID.IDLE
        self.states[self.state_id].on_enter()

    def find_path(self, start: StateID, goal: StateID) -> list[StateID] | None:
        """BFS по разрешённым рёбрам; guards проверяются при выполнении."""
        queue = deque([(start, [start])])
        visited = {start}
        while queue:
            current, path = queue.popleft()
            if current == goal:
                return path
            for src, dst in self.EDGES:
                if src == current and dst not in visited:
                    visited.add(dst)
                    queue.append((dst, path + [dst]))
        return None

    def request_state(self, target: StateID) -> bool:
        """Запрос устойчивого состояния (не мгновенная смена состояния)."""
        if target not in self.TARGETS:
            raise ValueError(f"Недопустимое целевое состояние: {target}")
        if (self.state_id == StateID.FAILURE or self.status.estop_active
                or self.safety_monitor.fault is not None):
            return False

        path = self.find_path(self.state_id, target)
        if path is None:
            return False

        self.target_state = target
        print("Маршрут:", " -> ".join(s.name for s in path))
        return True

    def _transition_to(self, next_state: StateID) -> bool:
        if next_state == self.state_id:
            return True

        # ESTOP должен обходить обычные проверки готовности/устойчивости.
        if next_state != StateID.FAILURE:
            if not self.states[self.state_id].can_exit(next_state):
                edge = (self.state_id, next_state)
                if self._last_blocked != edge:
                    print(f"Ожидание условий: {edge[0].name} -> {edge[1].name}")
                self._last_blocked = edge
                return False

        self._last_blocked = None
        if next_state == StateID.FAILURE:
            # Обычный on_exit() в аварии может выполнять небезопасные действия.
            self.states[self.state_id].on_abort()
        else:
            self.states[self.state_id].on_exit()
        self.state_id = next_state
        self.states[self.state_id].on_enter()
        return True

    def handle_event(self, event: Event) -> bool:
        # Глобальный переход из ЛЮБОГО состояния, включая промежуточные.
        if event == Event.ESTOP:
            self.target_state = None    # Без автоматического возобновления
            # Немедленно блокируем выходы ДО callbacks FSM.
            self.controller.emergency_stop()
            return self._transition_to(StateID.FAILURE)

        if event == Event.CLEAR_FAULT:
            if (self.state_id == StateID.FAILURE
                    and not self.status.estop_active
                    and self.safety_monitor.fault is None):
                self.target_state = None
                return self._transition_to(StateID.IDLE)
            return False

        if (self.status.estop_active or self.state_id == StateID.FAILURE
                or self.safety_monitor.fault is not None):
            return False

        for (src, dst), transition_event in self.EDGES.items():
            if src == self.state_id and transition_event == event:
                return self._transition_to(dst)
        return False

    def update(self) -> None:
        # Контроль безопасности ДО on_update и до отправки команд.
        # В IDLE отсутствие телеметрии блокирует запуск, но не вызывает ESTOP.
        feedback_required = self.state_id not in (
            StateID.IDLE, StateID.FAILURE
        )
        fault = self.safety_monitor.check(
            self.status.joint_feedback, feedback_required=feedback_required,
            fsm_state=self.state_id.name
        )
        if (self.status.estop_active or fault is not None) and self.state_id != StateID.FAILURE:
            if fault is not None:
                print(f"SAFETY: {fault.code}: {fault.description}")
            self.handle_event(Event.ESTOP)
            return

        event = self.states[self.state_id].on_update()
        if event is not None:
            self.handle_event(event)
            return

        # Переходные состояния завершаются своими on_update().
        if self.state_id in self.TRANSIENT or self.target_state is None:
            return
        if self.state_id == self.target_state:
            return

        path = self.find_path(self.state_id, self.target_state)
        if path is not None and len(path) > 1:
            next_state = path[1]
            self.handle_event(self.EDGES[(self.state_id, next_state)])


    def try_clear_fault(self, *, operator_acknowledged: bool) -> bool:
        """Clear only after manual acknowledgement and normal telemetry."""
        if self.state_id != StateID.FAILURE:
            return False
        if not self.safety_monitor.clear_fault(
            self.status.joint_feedback,
            operator_acknowledged=operator_acknowledged,
            external_estop_released=not self.status.estop_active,
        ):
            return False
        return self.handle_event(Event.CLEAR_FAULT)

    def enable_actuator_commands(self) -> bool:
        """Снять программную блокировку команд после готовности в IDLE.

        Реальное аппаратное включение приводов требует отдельного подтверждения.
        """
        if (self.state_id != StateID.IDLE
                or self.status.estop_active
                or not self.readiness_checker.is_ready(self.status)):
            return False
        self.controller.enable_outputs()
        return True


if __name__ == "__main__":
    # Только демонстрация. Для MORS задать ограничения всех 12 суставов.
    names = ("demo_hip", "demo_thigh", "demo_knee")
    monitor = SafetyMonitor(
        {name: JointLimits(-1.0, 1.0) for name in names},
        max_feedback_age_s=0.15,
    )
    readiness = ReadinessChecker(monitor, max_status_age_s=0.15,
                                 stable_time_s=0.10)
    status = RobotStatus()
    fsm = RobotFSM(status=status, safety_monitor=monitor,
                   readiness_checker=readiness)

    def feed(knee: float = 0.0) -> None:
        # Реальный робот обновляет поля только при приёме новых сообщений.
        now = time.monotonic()
        status.joint_feedback = JointFeedback(
            {"demo_hip": 0.0, "demo_thigh": 0.0, "demo_knee": knee},
            stamp_monotonic_s=now,
        )
        status.drive_status_stamp_s = now
        status.contact_stamp_s = now
        status.estimator_stamp_s = now

    print("\n1. Отсутствие данных не вызывает ESTOP, но блокирует выход из IDLE")
    assert fsm.request_state(StateID.STANDING)
    fsm.update()
    assert fsm.state_id == StateID.IDLE and monitor.fault is None
    print("Причины:", readiness.last_errors)

    print("\n2. После диагностики: IDLE -> STANDUP -> STANDING")
    status.drives_ready = status.contact_sensors_ready = status.estimator_ready = True
    # В IDLE и STANDUP положение за пределами разрешено.
    for _ in range(6):
        feed(knee=1.2)
        fsm.update()
        time.sleep(0.025)
    assert fsm.state_id == StateID.STANDUP and monitor.fault is None
    for _ in range(2):
        feed(knee=1.2)
        fsm.update()
    feed(knee=0.0)
    fsm.update()
    assert fsm.state_id == StateID.STANDING

    print("\n3. Превышение угла в STANDING -> ESTOP")
    feed(knee=1.2)
    fsm.update()
    assert fsm.state_id == StateID.FAILURE
    assert monitor.fault is not None and monitor.fault.code == "JOINT_LIMIT"
    assert fsm.controller.outputs_inhibited

    print("\n4. Ручной сброс и разрешение выдачи команд")
    feed()
    assert not fsm.try_clear_fault(operator_acknowledged=False)
    assert fsm.try_clear_fault(operator_acknowledged=True)
    assert fsm.state_id == StateID.IDLE and fsm.controller.outputs_inhibited
    for _ in range(6):
        feed()
        fsm.update()
        time.sleep(0.025)
    assert fsm.enable_actuator_commands()

    print("\n5. Потеря актуальных данных в STANDUP -> ESTOP")
    assert fsm.request_state(StateID.STANDING)
    feed()
    fsm.update()
    assert fsm.state_id == StateID.STANDUP
    time.sleep(0.16)
    fsm.update()
    assert fsm.state_id == StateID.FAILURE
    assert monitor.fault is not None and monitor.fault.code == "JOINT_FEEDBACK_STALE"
    print("\nВсе демонстрационные проверки пройдены.")
