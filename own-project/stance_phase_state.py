from motion_state_machine import State


class StancePhaseState(State):

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