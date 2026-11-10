from abc import ABC
from collections import deque
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


class State(ABC):
    def __init__(self, fsm):
        self.fsm = fsm

    def on_enter(self):
        print(f"Enter {type(self).__name__}")

    def on_update(self):
        # Каждое состояние может сигнализировать о неисправности.
        if self.fsm.estop_active:
            return Event.ESTOP
        return None

    def on_exit(self):
        print(f"Exit  {type(self).__name__}")


class IdleState(State):
    pass


class StandupState(State):
    def on_enter(self):
        super().on_enter()
        self.ticks = 0

    def on_update(self):
        event = super().on_update()
        if event is not None:
            return event
        self.ticks += 1
        if self.ticks >= 3:  # Здесь: проверка завершения вставания
            return Event.START_STANDING
        return None


class StandingState(State):
    pass


class LocomotionState(State):
    def on_update(self):
        event = super().on_update()
        if event is not None:
            return event
        print("LocomotionControl.step()")
        return None


class LaydownState(State):
    def on_enter(self):
        super().on_enter()
        self.ticks = 0

    def on_update(self):
        event = super().on_update()
        if event is not None:
            return event
        self.ticks += 1
        if self.ticks >= 3:  # Здесь: проверка завершения укладывания
            return Event.START_IDLE
        return None


class ActionState(State):
    pass


class ExternalState(State):
    pass


class FailureState(State):
    def on_enter(self):
        super().on_enter()
        print("FAILURE: аварийная остановка")


class RobotFSM:
    # Граф допустимых переходов со схемы: (откуда, куда) -> событие.
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

    # STANDUP и LAYDOWN — переходные состояния: нельзя их пропускать.
    TRANSIENT = {StateID.STANDUP, StateID.LAYDOWN}
    TARGETS = {
        StateID.IDLE, StateID.STANDING, StateID.LOCOMOTION,
        StateID.ACTION, StateID.EXTERNAL,
    }

    def __init__(self):
        self.estop_active = False
        self.target_state = None

        self.states = {
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

    def _find_path(self, start, goal):
        """BFS: кратчайший путь по числу переходов."""
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

    def request_state(self, target):
        """Команда от пользователя: к какому устойчивому режиму прийти."""
        if target not in self.TARGETS:
            raise ValueError(f"Недопустимое целевое состояние: {target}")
        if self.state_id == StateID.FAILURE or self.estop_active:
            print("Команда отклонена: робот в FAILURE / ESTOP")
            return False

        path = self._find_path(self.state_id, target)
        if path is None:
            print(f"Нет пути {self.state_id.name} -> {target.name}")
            return False

        self.target_state = target
        print("Маршрут:", " -> ".join(s.name for s in path))
        return True

    def _transition_to(self, next_state):
        if next_state == self.state_id:
            return
        self.states[self.state_id].on_exit()
        self.state_id = next_state
        self.states[self.state_id].on_enter()

    def handle_event(self, event):
        if event == Event.ESTOP:
            self.estop_active = True  # Защёлкнуть аварийный сигнал
            self.target_state = None  # Не возобновлять движение после сброса
            self._transition_to(StateID.FAILURE)
            return

        if event == Event.CLEAR_FAULT:
            if self.state_id == StateID.FAILURE and not self.estop_active:
                self._transition_to(StateID.IDLE)
            return

        for (src, dst), transition_event in self.EDGES.items():
            if src == self.state_id and transition_event == event:
                self._transition_to(dst)
                return

        print(f"Недопустимое событие {event.name} в {self.state_id.name}")

    def update(self):
        event = self.states[self.state_id].on_update()
        if event is not None:
            self.handle_event(event)
            return

        if self.state_id in self.TRANSIENT or self.target_state is None:
            return  # Ждём завершения STANDUP / LAYDOWN

        if self.state_id == self.target_state:
            return

        path = self._find_path(self.state_id, self.target_state)
        if path is not None and len(path) > 1:
            next_state = path[1]
            self.handle_event(self.EDGES[(self.state_id, next_state)])


if __name__ == "__main__":
    fsm = RobotFSM()

    fsm.request_state(StateID.STANDING)  # IDLE -> STANDUP -> STANDING
    for _ in range(5):
        fsm.update()

    fsm.request_state(StateID.LOCOMOTION)
    for _ in range(2):
        fsm.update()

    fsm.request_state(StateID.IDLE)  # Через STANDING и LAYDOWN
    for _ in range(6):
        fsm.update()

    fsm.request_state(StateID.LOCOMOTION)
    fsm.update()
    fsm.handle_event(Event.ESTOP)  # Из любого состояния -> FAILURE
    fsm.request_state(StateID.STANDING)  # Не принимается
    fsm.estop_active = False            # Имитация снятия сигнала ESTOP
    fsm.handle_event(Event.CLEAR_FAULT)  # FAILURE -> IDLE
