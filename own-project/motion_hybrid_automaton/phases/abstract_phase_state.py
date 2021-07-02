import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl
import numpy as np


class AbstractPhaseState:

    def __init__(self, hybrid_automaton, constraint, guard_functions):
        self.model = hybrid_automaton.model
        self.slip_model = hybrid_automaton.slip_model
        self.gui_plot = hybrid_automaton.gui_plot

        self.constraint = constraint
        self.events = guard_functions

    def controller_iteration(self, time, state):
        """
        performs iteration for a time step for the controller of this state
        :param time: time of the current iteration
        :param state: current continuous state vector of the robot
        :return: torque (tau) from the controller
        """
        pass  # this is implemented for the concrete states only

    def flow_function(self, time, x):
        """
        computes the generalized states derivative given the generalized states (velocities and forces during flight)
        :param time: time
        :param x: generalized coordinates and their derivatives y=[q, qd].transpose()
        :return: derivative yd = [qd, qdd]
        """
        from motion_hybrid_automaton.motion_hybrid_automaton import ContinuousState
        state = ContinuousState(self.model, x)

        tau_desired = self.controller_iteration(time, state)  # Controller

        self.plot_state(state, time)  # Visualization

        xd = self._forward_dynamics(tau_desired, state)  # System dynamics

        return xd  # derivative of the state [q, qd]

    def plot_state(self, state, time):
        if self.gui_plot:
            self.gui_plot.updateRobot(time, state.q)
            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, foot_id, np.zeros(3), True)[:2]
            self.gui_plot.showPoints(time, 'go-', state.pos_com(), foot_pos)
            self.gui_plot.showPoints(time, 'mo-', state.pos_com(), state.pos_com() + state.vel_com() * 1e-1)
            self.gui_plot.showPoints(time, 'ro-', state.pos_com(), state.pos_com() + self.slip_model.slip_force * 1e-3)

    def _forward_dynamics(self, tau_desired, state):
        """
        calculates the forward dynamics given tau from the controller, f_ext and the leg state q, qd
        :param tau_desired: the tau given from the controller
        :param state: state variables of the leg (q, qd)
        :return: qd, qdd
        """
        # calculate qdd from q, qd, tau, f_ext for the model with the forward dynamics
        # rbdl.ForwardDynamicsConstraintsDirect(self.leg_model, state.q, state.qd, state.tau,
        #   self.constraints[self.discrete_state], state.qdd) --> this is just done during stance
        qdd = np.zeros(self.model.qdot_size)
        rbdl.ForwardDynamicsConstraintsDirect(self.model, state.q, state.qd,
                                              tau_desired, self.constraint, qdd)

        # return res = [qd, qdd] from the forward dynamics which can then be given to the solver to be integrated by the ivp_solver
        xd = np.zeros(2 * self.model.dof_count)
        xd[:self.model.dof_count] = state.qd
        xd[self.model.dof_count:] = qdd
        return xd
