from flight_phase_state import FlightPhaseState
from stance_phase_state import StancePhaseState


class State:
    def check_for_transition(self):
        """
        checks if there is a transition to another state
        and returns it if there is
        :return: next state after transition
        """
        pass

    def run_solver(self):
        """
        run the solver for this state
        """


class MotionStateMachine:
    """
    This state machine represents the state of the walker,
    whether it is in stance or fight mode
    """

    def __init__(self):
        flight_state = FlightPhaseState()
        stance_state = StancePhaseState()
        self.states = {}
        self.states.put("Flight", flight_state)
        self.states.put("Stance", stance_state)
        self.active_state : State = self.states.get("Flight")

    def update_state(self):
        """
        Checks if there is a state transition
        and applies it to update the state of the state machine.
        :return:
        """
        next_state = self.active_state.check_for_transition()
        if next_state:
            self.active_state = self.states(next_state)

    def run_solver_for_active_state(self):
        """
        run the solver for the active state
        """
        self.active_state.run_solver()


