import math
from typing import Tuple, Any

import numpy as np
import rbdl

"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from math_model import calc_numerical_gradient, MathModel, State
from phase_state import PhaseState
class PIDController:
    """
    this class represent a PID controller with its gain values
    """

    def __init__(self, k_p, k_i, k_d, k_c=0, init_i_error=0):
        """
        :param k_p: propositional gain value of the PID controller
        :param k_i: integral gain value
        :param k_d: derivative gain value
        :param k_c: conditional gain value
        """
        self.i_error = init_i_error
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.k_c = k_c

    def control_function(self, p_error, d_error=0, c_error=0, scale_i_error=1):
        output = self.k_p * p_error + self.k_i * scale_i_error * self.i_error + self.k_d * d_error
        output += self.k_c * c_error
  
        return output


def limit_value_to_max_abs(value, max_abs):
    limited_value = np.minimum(max_abs, value)
    limited_value = np.maximum(-max_abs, limited_value)
    return limited_value


class FlightPhaseState(PhaseState):

    def __init__(self, math_model):
        super().__init__(math_model)

        def touchdown_event(time, y):
            """
            This event marks the contact of foot with ground.
            It is an event function for integrating solver of the flight phase.
            When touchdown_event(t,y) = 0 solver stops
            :param time: time
            :param y: generalized coordinates and their derivative y=[q, qd].transpose()
            :return:
            """
            self.math_model.state.q = y[:self.dof_count]
            self.math_model.state.qd = y[self.dof_count:]

            # Returns the base coordinates of the point (0,0,0) (--> origin of body frame) given in body coordinates of the foot
            foot_id = self.math_model.model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.math_model.model, self.leg_state.q, self.leg_model, np.zeros(3), True)
            return foot_pos[1]

        touchdown_event.terminal = True

        self.events = [touchdown_event]

        # Position PI Controller
        self.max_pos = 1
        self.max_vel = 1
        self.position_pi_ctr = PIDController(k_p=0.4, k_i=0.1, k_d=0)

        # Velocity adapted PI Controller
        self.max_angle_of_attack = 18  # in deg
        self.max_control_vel = 6
        self.velocity_pid_ctr = PIDController(k_p=4, k_i=3.5, k_d = 0, k_c=5) #TODO set k_d= 0 is ok if it is not used (PI controller)but needed in init?

        # Pose PID Controller
        self.local_leg_length_spring = 0.9
        self.i_max_control = 0.5 * np.ones((2))
        self.pose_pid_ctr = PIDController(k_p=1020, k_i=100, k_d=70, init_i_error=np.zeros(2))

        self.previous_pos_error = None
        self.set_forces = True
        self.set_forces_glob = False

    def controller_iteration(self, iteration_counter: int, timestep: float):
        """
        performs iteration for the controller of the flight phase.
        This simulates moves the leg to the right position to control velocity, position of the robot
        run the solver for the flight phase state. Calculates the xdd_des (desired acceleration) of the robot.
        :param iteration_counter: the current iteration number
        :param timestep: length of a time step / iteration
        :return:
        """

        # DISTURBED
        # do we need this
        # TODO contact_nr
        #if abs(self.math_model.vel_com[1]) < 0.01 and self.set_forces:  # and contact_nr == 3 :
        #    self.set_forces = False
        #    self.set_forces_glob = True

        des_pos = self.math_model.des_com_pos
        print("desired pos: %s" % des_pos)

        # Position Controller
        vel_des = self.position_controller(self.math_model, des_pos)

        # Velocity Controller
        angle_of_attack = self.velocity_controller(iteration_counter, self.math_model, vel_des)

        #
        xdd = self.pose_controller(self.math_model, timestep, angle_of_attack)

        #TODO is qr factorization the same as completeOrthogonalDecomposition in c++ code und richtige reihenfolge
        #TODO why do we need QR decomposition here?
        #q, r = np.linalg.qr(self.math_model.jac_base_s)
        pinv_jac_base_s = np.linalg.pinv(self.math_model.jac_base_s)
        
        tau_flight = self.math_model.mass_matrix @ pinv_jac_base_s @ xdd

        self.math_model.update()
        self.math_model.impact = False

        return tau_flight

    def pose_controller(self, math_model, timestep, angle_of_attack):
        # POSE/LEG CONTROLLER
        pos_foot_des = np.zeros(2)
        angle_of_attack_rad = np.deg2rad(angle_of_attack)
        pos_foot_des[0] = math_model.pos_com[0] + math.sin(angle_of_attack_rad) * self.local_leg_length_spring
        pos_foot_des[1] = math_model.pos_com[1] - math.cos(angle_of_attack_rad) * self.local_leg_length_spring

        # Proportional part PID
        foot_id = math_model.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(math_model.model, math_model.state.q, foot_id, np.zeros(3), True)[:2]

        pos_error = pos_foot_des - pos_foot
        # Derivative part PID
        vel_error = calc_numerical_gradient(pos_error, self.previous_pos_error, timestep)
        self.previous_pos_error = pos_error
        # Integral part PID
        self.pose_pid_ctr.i_error += pos_error
        
        self.pose_pid_ctr.i_error = limit_value_to_max_abs(self.pose_pid_ctr.i_error, self.i_max_control)

        # PID control


        mass_inv = np.linalg.inv(math_model.mass_matrix_ee)
        xdd = mass_inv @ self.pose_pid_ctr.control_function(p_error=pos_error, scale_i_error=timestep,
                                                                d_error=vel_error)
        return xdd

    def velocity_controller(self, iteration_counter, math_model, vel_des):
        cur_vel = math_model.vel_com[0]  # current velocity

        # Proportional Part PID
        p_error = (cur_vel - vel_des)

        # FIRST IMPACT
        if math_model.first_iteration_after_impact:
            math_model.first_iteration_after_impact = False
            math_model.leg_length_delta = 0

            # VELOCITY CONTROLLER
            # TODO do we need this
            # vel_com_start_flight = math_model.vel_com
            # vel_com_diff_phase = vel_com_start_flight - math_model.vel_com_start_stance
            
            # Integral Part PID
            self.velocity_pid_ctr.i_error += p_error  # accumalates the error - integral term

        self.velocity_pid_ctr.i_error = limit_value_to_max_abs(self.velocity_pid_ctr.i_error, self.max_control_vel)
        vel_com_x = math_model.vel_com[0]  # velocity of center of mass in x direction
        if iteration_counter % 5 == 0 or not math_model.angle_of_attack:
            # PID
            math_model.angle_of_attack = self.velocity_pid_ctr.control_function(p_error=p_error, c_error=vel_com_x)
            math_model.angle_of_attack = limit_value_to_max_abs(math_model.angle_of_attack, self.max_angle_of_attack)
        return math_model.angle_of_attack

    def position_controller(self, math_model, des_pos):
        
        cur_pos = math_model.pos_com[0]  # current position

        # Proportional Part PID
        p_error = (des_pos - cur_pos)
        # Integral Part PID
        self.position_pi_ctr.i_error += p_error
        self.position_pi_ctr.i_error = limit_value_to_max_abs(self.position_pi_ctr.i_error, self.max_pos)
        
        # PID
        vel_des = self.position_pi_ctr.control_function(p_error=p_error)
        vel_des = limit_value_to_max_abs(vel_des, self.max_vel)
        return vel_des

    def transfer_to_next_state(self, solver_result) -> Tuple[Any, Any]:
        """
        transfer flight to stance:
        transfers the solver_result to the state and start time for the next stance phase state
        :param solver_result: resulting output of the solver
        :return: a tuple of robot's state for the next phase and the end time (start time for the next state)
        """
        # set initial time t and state for next iteration which is last element of stored solver values
        t_impact = solver_result.t[-1]
        solver_state = solver_result.y.T[-1]
        # Calculation of change of velocity through impact
        # qd_minus is the generalized velocity before and qd_plus the generalized velocity after the impact
        q_minus = solver_state[:self.dof_count]  # first half of state vector is q
        qd_minus = solver_state[self.dof_count:]  # second half of state vector is qd
        qd_plus = np.ones(self.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.leg_model, q_minus, qd_minus, self.constraints, qd_plus)
        # replace generalized velocity in state vector for next iteration by calculated velocity after ground impact
        solver_state[self.dof_count:] = qd_plus
        return solver_state, t_impact