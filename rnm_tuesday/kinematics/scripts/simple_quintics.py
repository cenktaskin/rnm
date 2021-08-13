#!/usr/bin/env python3
# by cenkt

import numpy as np


class QuinticTrajectory:
    @staticmethod
    def position(t):
        return np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5])

    @staticmethod
    def velocity(t):
        return np.array([0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])

    @staticmethod
    def acceleration(t):
        return np.array([0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])

    @staticmethod
    def jerk(t):
        return np.array([0, 0, 0, 6, 24 * t, 60 * t ** 2])

    def quintic_mat(self, t):
        return np.array([self.position(t), self.velocity(t), self.acceleration(t), self.jerk(t)])

    def quintic_coefficient_matrix(self, t0, tf):
        return np.vstack([self.quintic_mat(t0)[:-1], self.quintic_mat(tf)[:-1]])

    def find_quintic_coefficients(self, q_i, q_t, duration):
        b = [q_i, 0, 0, q_t, 0, 0]
        return np.linalg.solve(self.quintic_coefficient_matrix(0, duration), b)

    def get_max_values_from_traj(self, coeffs, dur):
        max_vel_point = self.velocity(dur / 2)
        x_cric = 0.21132487  # Root of the jacobian scaled to the duration
        # Acceleration is a bit risky since the root changes with res, the point is not fixed
        max_acc_point = self.acceleration(dur * x_cric)
        max_jerk_point = self.jerk(0)
        max_vels = max_vel_point @ coeffs.T
        max_accs = max_acc_point @ coeffs.T
        max_jerks = max_jerk_point @ coeffs.T
        return max_vels, max_accs, max_jerks
