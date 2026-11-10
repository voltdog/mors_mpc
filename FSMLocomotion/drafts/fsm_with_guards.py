"""Пример FSM робота с BFS и проверками выхода из IDLE/LOCOMOTION.

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
    estop_active: bool = False


class LocomotionControl:
    """Заглушка: замените реальной реализацией LocomotionControl."""

    def __init__(self):
        self.velocity_command = (0.0, 0.0, 0.0)

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> None:
        self.velocity_command = (vx, vy, yaw_rate)

    def step(self) -> None:
        # Здесь вызывается ваш алгоритм управления движением.
        # RobotStatus обновляется сенсорами/оценщиком, а не этой заглушкой.
        pass

    def emergency_stop(self) -> None:
        # Демонстрация. На реальном роботе независимая защита обязательна.
        self.set_velocity(0.0, 0.0, 0.0)
        print("Controller: emergency stop requested")


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


class IdleState(State):
    def can_exit(self, next_state: StateID) -> bool:
        if next_state != StateID.STANDUP:
            return False

        s = self.fsm.status
        return (
            s.drives_ready
            and s.contact_sensors_ready
            and s.estimator_ready
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
    ) -> None:
        self.status = status if status is not None else RobotStatus()
        self.controller = controller if controller is not None else LocomotionControl()
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
        if self.state_id == StateID.FAILURE or self.status.estop_active:
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
        self.states[self.state_id].on_exit()
        self.state_id = next_state
        self.states[self.state_id].on_enter()
        return True

    def handle_event(self, event: Event) -> bool:
        # Глобальный переход из ЛЮБОГО состояния, включая промежуточные.
        if event == Event.ESTOP:
            self.target_state = None    # Без автоматического возобновления
            return self._transition_to(StateID.FAILURE)

        if event == Event.CLEAR_FAULT:
            if self.state_id == StateID.FAILURE and not self.status.estop_active:
                self.target_state = None
                return self._transition_to(StateID.IDLE)
            return False

        if self.status.estop_active or self.state_id == StateID.FAILURE:
            return False

        for (src, dst), transition_event in self.EDGES.items():
            if src == self.state_id and transition_event == event:
                return self._transition_to(dst)
        return False

    def update(self) -> None:
        # Авария проверяется ДО on_update любого состояния.
        if self.status.estop_active and self.state_id != StateID.FAILURE:
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


if __name__ == "__main__":
    # Условные данные с датчиков / оценщика состояния.
    status = RobotStatus(
        drives_ready=False,
        contact_sensors_ready=True,
        estimator_ready=True,
    )
    fsm = RobotFSM(status=status)

    print("\n1. Команда LOCOMOTION, но приводы не готовы")
    fsm.request_state(StateID.LOCOMOTION)
    fsm.update()
    assert fsm.state_id == StateID.IDLE

    print("\n2. Приводы готовы: IDLE -> STANDUP -> STANDING -> LOCOMOTION")
    status.drives_ready = True
    for _ in range(6):
        fsm.update()
    assert fsm.state_id == StateID.LOCOMOTION

    print("\n3. Запрос STANDING: ждём нулевую скорость и 4 контакта")
    status.vx = 0.4
    status.contacts = (True, False, False, True)
    status.gait_stopped = False
    fsm.request_state(StateID.STANDING)
    for _ in range(3):
        fsm.update()
    assert fsm.state_id == StateID.LOCOMOTION
    assert fsm.controller.velocity_command == (0.0, 0.0, 0.0)

    print("\n4. Робот остановился и встал на 4 ноги")
    status.vx = 0.01
    status.vy = 0.01
    status.yaw_rate = 0.01
    status.contacts = (True, True, True, True)
    status.gait_stopped = True
    for _ in range(10):
        fsm.update()
        if fsm.state_id == StateID.STANDING:
            break
        time.sleep(0.05)
    assert fsm.state_id == StateID.STANDING

    print("\n5. STANDING -> LAYDOWN -> IDLE")
    fsm.request_state(StateID.IDLE)
    for _ in range(5):
        fsm.update()
    assert fsm.state_id == StateID.IDLE

    print("\n6. ESTOP из IDLE и ручной сброс")
    status.estop_active = True
    fsm.update()
    assert fsm.state_id == StateID.FAILURE
    status.estop_active = False
    fsm.handle_event(Event.CLEAR_FAULT)
    assert fsm.state_id == StateID.IDLE
    print("\nГотово. Все проверки пройдены.")
