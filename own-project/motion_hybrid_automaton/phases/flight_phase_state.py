import math
from typing import Tuple, Any

import numpy as np
import sys
import pathlib

from motion_hybrid_automaton.continuous_state import ContinuousState

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl

from motion_hybrid_automaton.phases.abstract_phase_state import AbstractPhaseState


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
        """

        :param p_error: error for proportional part in PID
        :param d_error: error for differential part in PID
        :param c_error: TODO: value of which should be controlled and which is used for calculating error together with desired value
        :param scale_i_error: coefficent for integral part in PID
        :return: output of PID controller
        """
        output = self.k_p * p_error + self.k_i * scale_i_error * self.i_error + self.k_d * d_error
        output += self.k_c * c_error

        return output


def limit_value_to_max_abs(value, max_abs):
    """

    :param value: value which should be clipped
    :param max_abs: max value for clipping
    :return: clipped value
    """
    limited_value = np.minimum(max_abs, value)
    limited_value = np.maximum(-max_abs, limited_value)
    return limited_value


class FlightPhaseState(AbstractPhaseState):

    def __init__(self, hybrid_automaton, constraint, desired_pos_com, guard_functions):
        super().__init__(hybrid_automaton, constraint, guard_functions)
        self.des_pos_com = desired_pos_com  # goal foot position (1 dimensional - x axis)

        # Position PI Controller for controlling velocity of com based on the com position (outer cascade)
        self.max_pos = 1 #to clip the Integral part where the com position errors are summed up
        self.max_vel = 1 #to clip the velocity (output of outer cascade)
        self.position_pi_ctr = PIDController(k_p=0.4, k_i=0.1, k_d=0)

        # Velocity adapted PI Controller for controlling the angle of attack based on the com velocity (middle cascade)
        self.max_angle_of_attack = 18  # in deg #to clip the angle of attack (output of middle cascade)
        self.max_control_vel = 6 #to clip the Integral part where the com velocity errors are summed up
        self.velocity_pid_ctr = PIDController(k_p=4, k_i=3.5, k_d=0,k_c=5)  # TODO set k_d= 0 is ok if it is not used (PI controller)but needed in init?
        self.angle_of_attack = 0

        # Pose PID Controller for controlling the foot acceleration based on the angle of attack (inner cascade)
        self.local_leg_length_spring = 0.9  #controlls the desired leg length for the impact
        self.i_max_control = 0.5 * np.ones((2)) #to clip the Integral part where the foot position errors are summed up
        self.pose_pid_ctr = PIDController(k_p=200, k_i=0, k_d=-16, init_i_error=np.zeros(2))
        self.pos_error_grad = np.zeros(2) #differential foot position error
        self.previous_pos_error = None # last foot position error for numerical gradient

        self.iteration_counter = 0 #counts number of iterations called be intergrator


    def controller_iteration(self, time, state):
        """
        performs iteration for the controller of the flight phase.
        The goal is to control velocity, position of the robot
        With a cascade control (3 steps) the desired foot accelartion is calculated from there the tau can be calculated with inverse dynamics
        :param time: current time
        :param state: robot state
        :return: tau to control the robot torques
        """
        self.iteration_counter += 1

        # Position Controller
        vel_des = self.position_controller(state, self.des_pos_com)

        # Velocity Controller
        angle_of_attack = self.velocity_controller(state, vel_des)

        # Pose/Leg Controller
        xdd = self.pose_controller(time, state, angle_of_attack)

        # TODO is qr factorization the same as completeOrthogonalDecomposition in c++ code und richtige reihenfolge
        # TODO why do we need QR decomposition here?
        # q, r = np.linalg.qr(self.math_model.jac_base_s)

        # calculate tau during flight
        pinv_jac_base_s = np.linalg.pinv(state.jac_base_s())
        tau_flight = state.mass_matrix() @ pinv_jac_base_s @ xdd

        return tau_flight

    def position_controller(self, state: ContinuousState, des_pos):
        """
        Outer cascade calculates the error between current and desired com position and calculates from there the desired com velocity
        :param state: robot state
        :param des_pos: goal position of the com
        :return: desired com velocity
        """

        # Proportional Part PID
        cur_pos = state.pos_com()[0]  # current com position
        p_error = (des_pos - cur_pos)

        # Integral Part PID
        self.position_pi_ctr.i_error += p_error
        self.position_pi_ctr.i_error = limit_value_to_max_abs(self.position_pi_ctr.i_error, self.max_pos) #clip summed up com position error

        # PID
        vel_des = self.position_pi_ctr.control_function(p_error=p_error)
        vel_des = limit_value_to_max_abs(vel_des, self.max_vel) #clip output

        return vel_des

    def velocity_controller(self, state: ContinuousState, vel_des):
        """
        Middle cascade calculates the error between current and desired com velocity and calculates from there the desired angle of attack
        :param state: robot state
        :param vel_des: goal velocity of the com from outer cascade
        :return: desired angel of attack
        """

        # Proportional Part PID
        cur_vel = state.vel_com()[0]  # current velocity of com in x direction
        p_error = (cur_vel - vel_des)

        # Integral Part of PID
        self.velocity_pid_ctr.i_error = limit_value_to_max_abs(self.velocity_pid_ctr.i_error, self.max_control_vel) #clip summed up velocity error

        if self.iteration_counter % 5e14 == 0 or not self.angle_of_attack: # do not update controlled angle of attack every itertation to make PID control in inner cascade more stable
            # PID
            self.angle_of_attack = self.velocity_pid_ctr.control_function(p_error=p_error, c_error=cur_vel)
            self.angle_of_attack = limit_value_to_max_abs(self.angle_of_attack, self.max_angle_of_attack) #clip output
        return self.angle_of_attack


    def pose_controller(self, time, state: ContinuousState, angle_of_attack):
        """
        Inner cascade calculates the error between current and desired foot position and calculates from there the desired foot acceleration
        :param time: current time step for diverential part in PID
        :param state: robot state
        :param angle_of_attack: goal angle of attack from middle cascade
        :return: desired foot acceleration
        """
        angle_of_attack_rad = np.deg2rad(angle_of_attack)

        #calculate the desired foot position from the angle of atack calculated in middle cascade and desired leg length for impact
        pos_foot_des = np.zeros(2)
        pos_foot_des[0] = state.pos_com()[0] + math.sin(angle_of_attack_rad) * self.local_leg_length_spring
        pos_foot_des[1] = state.pos_com()[1] - math.cos(angle_of_attack_rad) * self.local_leg_length_spring

        # current foot position
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, foot_id, np.zeros(3), True)[:2]

        # Proportional part PID
        pos_error = pos_foot_des - pos_foot

        # Diverential part PID
        time_diff = time - self.last_iteration_time
        if time_diff > 0.01:
            self.pos_error_grad = self.calc_numerical_gradient(pos_error, self.previous_pos_error, time_diff)
        vel_error = self.pos_error_grad
        self.previous_pos_error = pos_error

        # Integral part PID
        self.pose_pid_ctr.i_error += pos_error
        self.pose_pid_ctr.i_error = limit_value_to_max_abs(self.pose_pid_ctr.i_error, self.i_max_control) #clip summed up foot position error

        # PID control
        jac_s = state.jac_s()
        mass_matrix_ee_inv = jac_s @ state.inv_mass_matrix() @ jac_s.T
        xdd = mass_matrix_ee_inv @ self.pose_pid_ctr.control_function(p_error=pos_error, scale_i_error=time_diff,
                                                                      d_error=vel_error)
        return xdd


