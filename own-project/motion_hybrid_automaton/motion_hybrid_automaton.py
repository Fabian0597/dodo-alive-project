import math
from enum import Enum

import numpy as np
import rbdl

from scipy.integrate import solve_ivp

"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from model.slip_model_params import SlipModelParameterization
from motion_hybrid_automaton.phases.flight_phase_state import FlightPhaseState
from motion_hybrid_automaton.phases.stance_phase_state import StancePhaseState

import logging


class DiscreteState(Enum):
    STANCE = 0
    FLIGHT = 1
    NOT_MOVING = 2


class ContinuousState:
    __com = None
    __robot_mass = None
    __jac = None
    __model = None

    def __init__(self, model, y=None):
        self.q = np.zeros(model.qdot_size)
        self.qd = np.zeros(model.qdot_size)

        self.__model = model

        if y is not None:
            self.q = y[:model.dof_count]
            self.qd = y[model.dof_count:]

    def pos_com(self):
        """
        calculate COM for the the given robot configuration
        """
        if self.__com is None:

            mass_com_x = 0
            mass_com_y = 0
            total_mass = 0
            for i in range(2, len(self.__model.mBodies)):
                com_i = self.__model.GetBody(i).mCenterOfMass
                com_i_base = rbdl.CalcBodyToBaseCoordinates(self.__model, self.q, i, com_i, False)
                mass_i = self.__model.GetBody(i).mMass
                # if max(abs(com_i_base[0:2] - self.q[0:2])) > 1:
                #    return None  # something is wrong
                mass_com_x += com_i_base[0] * mass_i
                mass_com_y += com_i_base[1] * mass_i
                total_mass += mass_i

            com_x = mass_com_x / total_mass
            com_y = mass_com_y / total_mass

            self.__com = np.array([com_x, com_y, 0.])
            """
            num_of_joints = len(self.__model.mBodies)
            cog_mass_weighted = np.zeros(3)
            mass_sum = 0
            for i in range(2, num_of_joints):
                cog_in_body = self.__model.mBodies[i].mCenterOfMass
                cog_in_world = rbdl.CalcBodyToBaseCoordinates(self.__model, self.q, i, cog_in_body, True)
                cog_mass_weighted += cog_in_world * self.__model.mBodies[i].mMass
                mass_sum = mass_sum + self.__model.mBodies[i].mMass

            self.__com = (cog_mass_weighted / mass_sum)
            """
        return self.__com

    def robot_mass(self):

        if self.__robot_mass is None:
            totalMass = 0
            for i in range(2, len(self.__model.mBodies)):
                mass_i = self.__model.GetBody(i).mMass
                totalMass += mass_i
            self.__robot_mass = totalMass

        return self.__robot_mass

    def jac_com(self):
        """
        calculate Jacobian in COM frame for the the given robot configuration
        """
        if self.__jac is None:
            J_cog = np.zeros((3, self.__model.q_size))
            totalMass = 0
            for i in range(2, len(self.__model.mBodies)):
                com_i = self.__model.GetBody(i).mCenterOfMass
                totalMass += self.__model.GetBody(i).mMass
                jac_i = np.zeros((3, self.__model.q_size))
                rbdl.CalcPointJacobian(self.__model, self.q, i, com_i, jac_i, False)
                J_cog += self.__model.GetBody(i).mMass * jac_i
            J_cog /= totalMass
            self.__jac = J_cog
        return self.__jac

    def vel_com(self):
        """
        calculate velocity of COM frame for the the given robot configuration
        """
        jac = self.jac_com()
        return jac @ self.qd

    def to_array(self):
        """
        return the q and qd values stored as attributes in the ContinuousState object as as concatenated vector x
        """
        return np.concatenate(self.q, self.qd)


class GuardFunctions:

    def __init__(self, model, math_model):
        self.model = model
        self.math_model = math_model

        self.vel_threshold = 0.1
        self.lastSpringForce = -10000

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

            q = x[:self.model.dof_count]
            qd = x[self.model.dof_count:]
            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, q, foot_id, np.zeros(3), True)[:2]
            foot_y = foot_pos[1]

            base_vel_y = qd[1]
            if base_vel_y <= 0:  # make sure that the leg moves towards the ground
                return foot_y
            else:
                return 1

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

            q = x[:self.model.dof_count]
            qd = x[self.model.dof_count:]

            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, q, foot_id, np.zeros(3), True)  # [:2]

            state = ContinuousState(self.model, x)
            distance_foot_com = state.pos_com() - foot_pos
            slip_new_length = np.linalg.norm(distance_foot_com, ord=2)

            base_vel_y = qd[1]
            if base_vel_y >= 0:  # make sure that the leg moves away from the ground
                return self.math_model.slip_length - slip_new_length
            else:
                return 1

        g.terminal = True
        return g

    def exit_guard_function(self):
        def g(time, x):
            q = x[:self.model.dof_count]
            qd = x[self.model.dof_count:]

            foot_id = self.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, q, foot_id, np.zeros(3), True)[:2]

            vel_y = qd[1]
            foot_y = foot_pos[1]

            if foot_y == 0 and abs(vel_y) < self.vel_threshold:
                return 0
            else:
                return 1

        g.terminal = True
        return g


class JumpFunctions:

    def __init__(self, model, math_model, flight_constraint, stance_constraint):
        self.model = model
        self.math_model = math_model
        self.flight_constraint = flight_constraint
        self.stance_constraint = stance_constraint
        self.slip_length_zero = 0.6

    def _calculate_new_slip_length(self, time, state):
        # TODO check the Force is towards ground at the beginning of stance phase
        # TODO remove unused lines
        jac_com = state.jac_com()
        robot_mass = state.robot_mass()
        jac_s = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), jac_s, False)
        jac_s = np.delete(jac_s, 2, 0)
        # mass matrix
        mass_matrix = np.zeros((self.model.q_size, self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, state.q, mass_matrix, False)
        mass_matrix_inv = np.linalg.inv(mass_matrix)
        lambda_s = np.linalg.inv(jac_s @ mass_matrix_inv @ jac_s.T)
        null_s = np.eye(4) - mass_matrix_inv @ jac_s.T @ lambda_s @ jac_s
        matrix = jac_com.T @ jac_com - null_s.T @ jac_com.T @ jac_com @ null_s

        # post_vel = np.linalg.norm(jac_com @ null_s @ np.linalg.pinv(jac_com) @ state.vel_com(), ord=2)
        new_length = (1 / self.math_model.slip_stiffness) * robot_mass * (
                state.qd.T @ matrix @ state.qd)
        new_length = math.sqrt(abs(new_length))
        self.math_model.slip_length = self.slip_length_zero + new_length
        # energy = 0.5 * robot_mass * state.qd.T @ matrix @ state.qd

    def flight_to_stance_jump_function(self, time, x):
        # Calculation of change of velocity through impact
        # qd_minus is the generalized velocity before and qd_plus the generalized velocity after the impact
        """
        q_minus = x[:self.model.dof_count]  # first half of state vector is q
        qd_minus = x[self.model.dof_count:]  # second half of state vector is qd
        qd_plus = np.ones(self.model.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, q_minus, qd_minus, self.constraint, qd_plus)
        # replace generalized velocity in state vector for next iteration by calculated velocity after ground impact
        x[self.model.dof_count:] = qd_plus
        """
        logging.info("change phase: flight > stance (time: %s)" % time)

        # take robot snapshot
        state = ContinuousState(self.model, x)
        # compute impulses
        self._calculate_new_slip_length(time, state)
        # vel = np.linalg.norm(state.com_vel(), ord=2)
        qd_plus = np.ones(self.model.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, x[:self.model.dof_count], x[self.model.dof_count:],
                                             self.stance_constraint,
                                             qd_plus)
        x[self.model.dof_count:] = qd_plus
        return time, x

    def stance_to_flight_jump_function(self, time, x):
        logging.info("change phase: stance > flight (time: %s)" % time)
        return time, x


class MotionHybridAutomaton:
    """
    This hybrid automaton/state machine represents the states of the walker and the transitions to switch between states.
    The state consists of a discrete state (z) and a continuous state (x).
    z: whether it is in stance or fight mode
    x: position / velocity
    """

    def __init__(self, leg_model, des_com_pos, constraint_flight, constraint_stance, gui_plot):
        logging.info("Init Motion Hybrid Automaton")
        logging.debug("Create and update math model")
        self.model = leg_model
        self.slip_model = SlipModelParameterization()
        self.gui_plot = gui_plot

        guard = GuardFunctions(self.model, self.slip_model)
        jump = JumpFunctions(self.model, self.slip_model, constraint_flight, constraint_stance)

        logging.debug("Create State Phases")
        self.states = {
            DiscreteState.FLIGHT: FlightPhaseState(self, constraint_flight,
                                                   [guard.flight_to_stance_guard_function()]),
            DiscreteState.STANCE: StancePhaseState(self, constraint_stance,
                                                   [guard.stance_to_flight_guard_function()]),
        }

        logging.debug("Create Transitions")
        self.transitions = {
            DiscreteState.FLIGHT: [DiscreteState.STANCE, jump.flight_to_stance_jump_function],
            DiscreteState.STANCE: [DiscreteState.FLIGHT, jump.stance_to_flight_jump_function],
        }

        self.z = DiscreteState.FLIGHT  # discrete state
        # self.x = init_state  # continuous state

    def current_constraint(self):
        return self.states[self.z].constraint

    def simulate_motion(self, t_final, init_state, log_callback):
        """
        Calculates the differential equation of the movement for the stance and flight phase in sections alternately
        and logs the movement so that it can be visualised later in MeshUp.
        :param t_final: time when the solver should stop calculating the motion
        :param init_state: the initial state of the robot leg at time (t=0)
        :param log_callback: this method is called for logging a single time point with corresponding state
        """
        time, x = 0, init_state
        logging.info("Start motion simulation")

        while time < t_final:
            # Solve an initial value problem for a system of ordinary differential equations (ode)
            # Interval of integration (t_init,, t_final).
            # state = initial state
            # function to be integrated = f
            # it solves the ode until it reaches the event touchdown_event(y,t)=0
            # so the foot hits the ground and has (y height 0)
            active_state = self.states[self.z]
            # TODO: use t_eval ?
            # np.arange(time, t_final, 0.05)
            # t_eval = np.array([0.4, 0.7]).copy(order='C')
            # print(t_eval)
            solver = solve_ivp(
                fun=active_state.flow_function,
                t_span=[time, t_final], y0=x,
                max_step=0.2,
                events=active_state.events
            )
            time, x = solver.t[-1], solver.y.T[-1]

            # iterate over internal states saved by the solver during integration
            for i in range(0, len(solver.t)):
                # returns the value of the solution y=[q,qd] at time stamp T[i] from the interval of integration
                time = solver.t[i]
                x_t = solver.y.T[i][:self.model.dof_count]  # state at time T[i] without derivatives
                log_callback(time, x_t)

            # transition to next discrete state
            # (continuous state x is processed in the jump function to enter new discrete state)
            self.z, jump_function = self.transitions[self.z]
            time, x = jump_function(time, x)
