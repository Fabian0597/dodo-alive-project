from typing import Tuple, Any

import sys
import pathlib



basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl
import numpy as np
from math_model import MathModel, State


class PhaseState:

    def __init__(self, math_model: MathModel, constraint, plotter, event):
        self.iteration_counter = 0
        self.last_timestep = None
        self.math_model = math_model
        self.constraint = constraint
        self.plotter = plotter
        self.events = [event]

    def controller_iteration(self, time, state):
        """
        performs iteration for a time step for the controller of this state
        :param iteration_counter: iteration for counter of the solver
        :param timestep: timestep of the current iteration
        :return: torque (tau) from the controller (state of the robot's leg)
        """
        pass  # this is implemented for the concrete states only

    def transfer_to_next_state(self, solver_result) -> Tuple[Any, Any]:
        """
        this function maps the output of the solver of this state to the information for the next state
        :param solver_result: resulting output of the solver
        :return: a tuple of robot's state for the next phase and the end time (start time for the next state)
        """
        pass  # this is implemented for the concrete states only

    def flow_function(self, time, y):
        """
        computes the generalized states derivative given the generalized states (velocities and forces during flight)
        :param time: time
        :param y: generalized coordinates and their derivatives y=[q, qd].transpose()
        :return: derivative yd = [qd, qdd]
        """
        self.iteration_counter += 1
        delta_time = self._get_delta_time(time)
        self.math_model.state = State.from_q_qd_array(y, self.math_model.model.dof_count)
        self.math_model.new_timestep_update(delta_time)

        from motion_state_machine import ContinuousState
        state = ContinuousState(self.math_model.model, y)

        tau_desired = self.controller_iteration(time, state)  # Controller

        self.plot_state(state, time)  # Visualization

        state_derivative = self._forward_dynamics(tau_desired)  # System dynamics

        return state_derivative

    def plot_state(self, state, time):
        self.plotter.updateRobot(time, state.q)
        foot_id = self.math_model.model.GetBodyId('foot')
        foot_pos = rbdl.CalcBodyToBaseCoordinates(self.math_model.model, state.q, foot_id, np.zeros(3), True)[:2]
        self.plotter.showPoints(time, 'go-', state.pos_com(), foot_pos)
        self.plotter.showPoints(time, 'mo-', state.pos_com(), state.pos_com() + state.vel_com() * 1e-1)
        self.plotter.showPoints(time, 'ro-', state.pos_com(), state.pos_com() + self.math_model.ff * 1e-3)

    def _forward_dynamics(self, tau_desired):
        """
        calculates the forward dynamics given tau from the controller, f_ext and the leg state q, qd
        :param leg_state: state variables of the leg (q, qd, tau)
        :return: qd, qdd
        """
        # calculate qdd from q, qd, tau, f_ext for the model with the forward dynamics
        # rbdl.ForwardDynamicsConstraintsDirect(self.leg_model, state.q, state.qd, state.tau,
        #   self.constraints[self.discrete_state], state.qdd) --> this is just done during stance
        qdd = np.zeros(self.math_model.model.qdot_size)
        rbdl.ForwardDynamicsConstraintsDirect(self.math_model.model, self.math_model.state.q, self.math_model.state.qd,
                             tau_desired, self.constraint, qdd)

        # return res = [qd, qdd] from the forward dynamics which can then be given to the solver to be integrated by the ivp_solver
        state = np.zeros(2 * self.math_model.model.dof_count)
        state[:self.math_model.model.dof_count] = self.math_model.state.qd
        state[self.math_model.model.dof_count:] = qdd
        return state

    def _get_delta_time(self, time):
        if self.last_timestep == None:
            delta_time = 1e-12  # 0
        else:
            delta_time = time - self.last_timestep
        self.last_timestep = time
        return delta_time
        #return 1e-12 #TODO: time received from ivp_solver decreases with time and therfore does also delta_time decreases over time