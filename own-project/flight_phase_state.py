import math

import numpy as np
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-orb/build/python/')
import rbdl

from math_model import MathModel, calc_numerical_gradient
from motion_state_machine import State


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


class FlightPhaseState(State):

    def __init__(self):

        # Position PI Controller
        self.max_pos = 1
        self.max_vel = 1
        self.position_pi_ctr = PIDController(k_p=0.4, k_i=0.1, k_d=0)

        # Velocity adapted PI Controller
        self.max_angle_of_attack = 18  # in deg
        self.max_control_vel = 6
        self.velocity_pid_ctr = PIDController(k_c=5, k_p=4, k_i=3.5)

        # Pose PID Controller
        self.local_leg_length_spring = 0.9
        self.i_max_control = 0.5 * np.ones((2, 1))
        self.pose_pid_ctr = PIDController(k_p=1020, k_i=100, k_d=70, init_i_error=np.zeros((2, 1)))

        self.previous_pos_error = np.zeros((2, 1))
        self.set_forces = True
        self.set_forces_glob = False

    def run_solver(self, iteration_counter: int, timestep: float, math_model: MathModel):
        """
        run the solver for the flight phase state. Calculates the xdd_des (desired acceleration) of the robot.
        :param iteration_counter: the current iteration number
        :param timestep: length of a time step / iteration
        :param math_model: reference to the math model, where all the variables of the mathematical model
            of the robot are stored
        :return:
        """

        # DISTURBED
        # TODO contact_nr
        if abs(math_model.vel_com[1]) < 0.01 and contact_nr == 3 and self.set_forces:
            self.set_forces = False
            self.set_forces_glob = True

        p_error = self.position_controller(math_model)

        angle_of_attack = self.velocity_controller(iteration_counter, math_model, p_error)

        xdd = self.pose_controller(math_model, timestep, angle_of_attack)

        # TODO what to do with xdd and tau ?
        tau_flight = self.tau_update(xdd)

        #TODO: update tau in math_model_state here?
        math_model.state.tau = tau_flight

        math_model.update()
        # TODO x_new = solverFlightPhase.integrate(xVector, dt);
        math_model.impact = False

    def pose_controller(self, math_model, timestep, angle_of_attack):
        # POSE/LEG CONTROLLER
        pos_foot_des = np.zeros((2, 1))
        angle_of_attack_rad = math.radians(angle_of_attack)
        pos_foot_des[0] = math_model.pos_com[0] + math.sin(angle_of_attack_rad) * self.local_leg_length_spring
        pos_foot_des[1] = math_model.pos_com[1] - math.cos(angle_of_attack_rad) * self.local_leg_length_spring
        # Proportional part PID
        foot_id = math_model.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(math_model.model, math_model.state.q, foot_id, np.zeros(3), True)
        pos_error = pos_foot_des - pos_foot
        # Derivative part PID
        vel_error = calc_numerical_gradient(pos_error, self.previous_pos_error, timestep)
        self.previous_pos_error = pos_error
        # Integral part PID
        self.pose_pid_ctr.i_error += pos_error
        self.pose_pid_ctr.i_error = limit_value_to_max_abs(self.pose_pid_ctr.i_error, self.i_max_control)
        # PID control
        mass_inv = np.linalg.inv(math_model.mass_matrix_ee)
        xdd = mass_inv * self.pose_pid_ctr.control_function(p_error=pos_error, scale_i_error=timestep,
                                                                d_error=vel_error)
        return xdd

    def velocity_controller(self, iteration_counter, math_model, p_error):
        # FIRST IMPACT
        if math_model.first_iteration_after_impact:
            math_model.first_iteration_after_impact = False
            math_model.leg_length_delta = 0

            # VELOCITY CONTROLLER
            # TODO do we need this
            # vel_com_start_flight = math_model.vel_com
            # vel_com_diff_phase = vel_com_start_flight - math_model.vel_com_start_stance
            self.velocity_pid_ctr.i_error += p_error  # accumalates the error - integral term
        # VELOCITY CONTROLLER
        self.velocity_pid_ctr.i_error = limit_value_to_max_abs(self.velocity_pid_ctr.i_error, self.max_control_vel)
        vel_com_x = math_model.vel_com[0]  # velocity of center of mass in x direction
        if iteration_counter % 5 == 0:
            math_model.angle_of_attack = self.velocity_pid_ctr.control_function(p_error=p_error, c_error=vel_com_x)
        math_model.angle_of_attack = limit_value_to_max_abs(math_model.angle_of_attack, self.max_angle_of_attack)
        return math_model.angle_of_attack

    def position_controller(self, math_model):
        # POSITION CONTROLLER (control velocity to achieve a desired target)
        des_pos = None  # TODO: desired_xCom
        cur_pos = math_model.pos_com[0]  # current position
        cur_vel = math_model.vel_com[0]  # current velocity
        p_error = (des_pos - cur_pos)
        self.position_pi_ctr.i_error += p_error
        self.position_pi_ctr.i_error = limit_value_to_max_abs(self.position_pi_ctr.i_error, self.max_pos)
        vel_des = self.position_pi_ctr.control_function(p_error=p_error)
        vel_des = limit_value_to_max_abs(vel_des, self.max_vel)
        p_error = (cur_vel - vel_des)
        return p_error

    def tau_update(self, xdd):
        #TODO is qr factorization the same as completeOrthogonalDecomposition in c++ code und richtige reihenfolge
        # von qr und pinv
        pinv_jac_base_s = np.linalg.pinv(np.linalg.qr(math_model.jac_base_s))
        tau = math_model.mass_matrix * pinv * xdd
        return tau