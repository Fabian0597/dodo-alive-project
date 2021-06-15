import rbdl

from dynamics import MathModel
from flight_phase_state import FlightPhaseState
from stance_phase_state import StancePhaseState


class State:

    def run_solver(self, math_model: MathModel):
        """
        run the solver for this state
        """
        pass


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
        self.active_state: State = self.states.get("Flight")

    def check_for_state_transition(self, math_model: MathModel):
        """
        Checks if there is a state transition to another state
        and applies it to update the active state of the state machine.
        """
        foot_id = math_model.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(math_model.model, math_model.state.q, foot_id, np.zeros(3), True)
        if pos_foot[1] > 0:  # foot above ground
            next_state = "Flight"
        elif math_model.leg_spring_delta < 0 and math_model.vel_com[1] > 0:
            next_state = "Flight"
            # TODO impactCom = actualCom; updateCalculation()
        else:
            next_state = "Flight"
        self.active_state = self.states(next_state)

    def run_solver_for_active_state(self, math_model: MathModel):
        """
        run the solver for the active state
        """
        self.active_state.run_solver(math_model)


