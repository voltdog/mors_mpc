from abc import ABC
from enum import Enum, auto


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


class StateID(Enum):
    IDLE = auto()
    STANDUP = auto()
    STANDING = auto()
    LOCOMOTION = auto()
    LAYDOWN = auto()
    ACTION = auto()
    EXTERNAL = auto()
    FAILURE = auto()


# Базовый класс состояния
class State(ABC):
    def __init__(self, fsm):
        self.fsm = fsm

    def on_enter(self):
        print(f"Enter {type(self).__name__}")

    def on_update(self):
        # Любое состояние может запросить ESTOP
        if self.fsm.estop_requested:
            return Event.ESTOP
        return None

    def on_exit(self):
        print(f"Exit {type(self).__name__}")


class IdleState(State):
    def on_update(self):
        return super().on_update()


class StandupState(State):
    def on_enter(self):
        super().on_enter()
        self.counter = 0

    def on_update(self):
        event = super().on_update()
        if event:
            return event

        self.counter += 1

        # Пример завершения процедуры вставания
        if self.counter >= 3:
            return Event.START_STANDING

        return None


class StandingState(State):
    def on_update(self):
        return super().on_update()


class LocomotionState(State):
    def on_update(self):
        event = super().on_update()
        if event:
            return event

        print("LocomotionControl.step()")
        return None


class LaydownState(State):
    def on_enter(self):
        super().on_enter()
        self.counter = 0

    def on_update(self):
        event = super().on_update()
        if event:
            return event

        self.counter += 1

        # Пример завершения процедуры укладывания
        if self.counter >= 3:
            return Event.START_IDLE

        return None


class ActionState(State):
    def on_update(self):
        return super().on_update()


class ExternalState(State):
    def on_update(self):
        return super().on_update()


class FailureState(State):
    def on_enter(self):
        super().on_enter()
        print("Emergency stop: disable actuators!")

    def on_update(self):
        return super().on_update()


# Конечный автомат
class RobotFSM:

    TRANSITIONS = {
        (StateID.IDLE, Event.START_STANDUP):
            StateID.STANDUP,

        (StateID.STANDUP, Event.START_STANDING):
            StateID.STANDING,

        (StateID.STANDING, Event.START_LAYDOWN):
            StateID.LAYDOWN,

        (StateID.LAYDOWN, Event.START_IDLE):
            StateID.IDLE,

        (StateID.STANDING, Event.START_LOCOMOTION):
            StateID.LOCOMOTION,

        (StateID.LOCOMOTION, Event.START_STANDING):
            StateID.STANDING,

        (StateID.STANDING, Event.START_ACTION):
            StateID.ACTION,

        (StateID.ACTION, Event.START_STANDING):
            StateID.STANDING,

        (StateID.STANDING, Event.START_EXTERNAL):
            StateID.EXTERNAL,

        (StateID.EXTERNAL, Event.START_STANDING):
            StateID.STANDING,

        (StateID.FAILURE, Event.CLEAR_FAULT):
            StateID.IDLE,
    }

    def __init__(self):
        self.estop_requested = False

        # Все состояния создаются один раз
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
        self.state = self.states[self.state_id]
        self.state.on_enter()

    def transition_to(self, state_id):
        if state_id == self.state_id:
            return

        self.state.on_exit()

        self.state_id = state_id
        self.state = self.states[state_id]

        self.state.on_enter()

    def handle_event(self, event):
        # Глобальный переход из любого состояния
        if event == Event.ESTOP:
            self.transition_to(StateID.FAILURE)
            return

        # Нельзя сбросить аварию при активном ESTOP
        if event == Event.CLEAR_FAULT and self.estop_requested:
            print("Cannot clear fault while ESTOP is active")
            return

        # Поиск разрешённого перехода
        target = self.TRANSITIONS.get(
            (self.state_id, event)
        )

        if target is not None:
            self.transition_to(target)
        else:
            print(
                f"Ignored {event.name} "
                f"in {self.state_id.name}"
            )

    def update(self):
        event = self.state.on_update()

        if event is not None:
            self.handle_event(event)


# Пример использования
if __name__ == "__main__":
    fsm = RobotFSM()

    fsm.handle_event(Event.START_STANDUP)

    for _ in range(3):
        fsm.update()  # STANDUP -> STANDING

    fsm.handle_event(Event.START_LOCOMOTION)
    fsm.update()

    # Аварийная остановка из LOCOMOTION
    fsm.estop_requested = True
    fsm.update()  # LOCOMOTION -> FAILURE

    # Снятие аварии
    fsm.estop_requested = False
    fsm.handle_event(Event.CLEAR_FAULT)