import math

import numpy as np
import rbdl

from math_model import MathModel, calc_numerical_gradient
from motion_state_machine import State


class FlightPhaseState(State):

    def __init__(self):
        self.max_pos = 1
        self.max_vel = 1
        self.max_control_vel = 6
        self.max_angle_of_attack = 18  # in deg
        # Leg PID Controller
        self.i_max_control = 0.5
        self.integral_pos_error = np.zeros((2, 1))

        self.p_vel = 0.4
        self.i_vel_pos = 0.1

        self.total_error_pos
        self.total_error_vel
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

        # POSITION CONTROLLER
        des_pos = None  # TODO: desired_xCom
        error_pos = des_pos - math_model.center_of_mass[0]

        self.total_error_pos += error_pos
        vel_des = self.p_vel * error_pos + self.i_vel_pos * self.total_error_pos

        vel_des = min(self.max_pos, vel_des)
        vel_des = max(-self.max_pos, vel_des)

        error = math_model.vel_com[0] - vel_des

        # DISTURBED
        if abs(math_model.vel_com[1]) < 0.01 and contact_nr == 3 and self.set_forces:
            self.set_forces = False
            self.set_forces_glob = True

        # FIRST IMPACT
        if math_model.first_iteration_after_impact:
            math_model.first_iteration_after_impact = False
            math_model.leg_length_delta = 0
            # VELOCITY CONTROLLER
            vel_com_start_flight = math_model.vel_com
            vel_com_diff_phase = vel_com_start_flight - math_model.vel_com_start_stance
            self.total_error_vel += error  # accumalates the error - integral term

        # VELOCITY CONTROLLER
        self.total_error_vel = min(self.max_control_vel, self.total_error_vel)
        self.total_error_vel = max(-self.max_control_vel, self.total_error_vel)

        self.i_vel = 3.5
        self.c = 5.0
        self.k = 4

        if iteration_counter % 5 == 0:
            math_model.angle_of_attack = self.c * math_model.vel_com[
                0] + self.k * error + self.i_vel * self.total_error_vel

        math_model.angle_of_attack = min(self.max_angle_of_attack, math_model.angle_of_attack)
        math_model.angle_of_attack = max(-self.max_angle_of_attack, math_model.angle_of_attack)

        # LEG CONTROLLER
        local_leg_length_spring = 0.9

        pos_foot_des = np.zeros((2, 1))
        angle_of_attack_rad = math.radians(math_model.angle_of_attack)
        pos_foot_des[0] = math_model.center_of_mass[0] + math.sin(angle_of_attack_rad) * local_leg_length_spring
        pos_foot_des[1] = math_model.center_of_mass[1] - math.cos(angle_of_attack_rad) * local_leg_length_spring

        # Proportional part PID
        self.k_p = 1020
        foot_id = math_model.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(math_model.model, math_model.state.q, foot_id, np.zeros(3), True)
        pos_error = pos_foot_des - pos_foot
        # Derivative part PID
        self.k_d = 70
        vel_error = calc_numerical_gradient(pos_error, self.previous_pos_error, timestep)
        self.previous_pos_error = pos_error
        # Integral part PID
        self.k_i = 100
        self.integral_pos_error += pos_error
        self.integral_pos_error = np.minimum(self.i_max_control * np.ones(2), self.integral_pos_error)
        self.integral_pos_error = np.maximum(- self.i_max_control * np.ones(2), self.integral_pos_error)
        # PID control
        mass_inv = np.linalg.inv(math_model.mass_matrix_ee)
        xdd_des = mass_inv * (self.k_p * pos_error + self.k_d * vel_error + self.k_i * timestep * self.integral_pos_error)

        math_model.update()
        # TODO x_new = solverFlightPhase.integrate(xVector, dt);
        math_model.impact = False
