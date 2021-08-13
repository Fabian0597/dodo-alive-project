import numpy as np
import rbdl

from motion_hybrid_automaton.continuous_state import ContinuousState


class GuardFunctions:
    """
    defines events which are used by ivp_solver which activates the transition between states
    """

    def __init__(self, model, slip_model):
        self.model = model  # Lua leg model
        self.slip_model = slip_model  # parameter of leg model

        self.vel_threshold = 0.1  # threshold for velocity check whether foot is moving towards or away from ground

    def flight_to_stance_guard_function(self):
        def g(time, x):
            """
            calculates whether the given state activates the transition flight to stance
            This event marks the contact of foot with ground.
            It is an event function for integrating solver of the flight phase.
            When touchdown_event(t,y) = 0 solver stops
            :param time: time of the state
            :param x: continuous state of the robot [q, qd]
            :return: 0 if the transition is active
            """

            # generalized coordinates and velocities
            q = x[:self.model.dof_count]
            qd = x[self.model.dof_count:]

            # position of foot
            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, q, foot_id, np.zeros(3), True)[:2]
            foot_y = foot_pos[1]

            base_vel_y = qd[1]
            if base_vel_y <= 0:  # make sure that the leg moves towards the ground
                return foot_y  # returns 0 if foot hits the ground
            else:
                return 1  # return 1 in order to not launch the event

        g.terminal = True
        return g

    def stance_to_flight_guard_function(self):
        def g(time, x):
            """
            calculates whether the given state activates the transition flight to stance
            This event marks the transition from stance phase to fight phase, when the leg not touches the ground anymore.
            It is an event function for integrating solver of the stance phase.
            When touchdown_event(t,y) = 0 solver stops
            :param time: time of the state
            :param x: continuous state of the robot [q, qd]
            :return: 0 if the transition is active
            """

            # generalized coordinates and velocities
            q = x[:self.model.dof_count]
            qd = x[self.model.dof_count:]

            # foot position
            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, q, foot_id, np.zeros(3), True)  # [:2]

            # calculate leg length which corresponds to spring length in SLIP model
            state = ContinuousState(self.model, x)
            distance_foot_com = state.pos_com() - foot_pos
            slip_new_length = np.linalg.norm(distance_foot_com, ord=2)

            base_vel_y = qd[1]
            if base_vel_y >= 0:  # make sure that the leg moves away from the ground
                return self.slip_model.slip_length - slip_new_length  # returns 0 if the spring length is bigger than l0
            else:
                return 1  # return 1 in order to not launch the event

        g.terminal = True
        return g
