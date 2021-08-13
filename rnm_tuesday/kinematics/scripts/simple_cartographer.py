#!/usr/bin/env python3
# by cenkt

import numpy as np


class Cartographer:
    def look_at_the_point(self, pose0, point0, point1):
        z_e = self.normalize_vector(point1 - point0)
        x_0 = pose0[:3, 0]
        x_e = self.normalize_vector(x_0 - (z_e @ x_0) * z_e)
        y_e = np.cross(z_e, x_e)
        pose1 = np.eye(4)
        pose1[:-1, :-1] = np.vstack([x_e, y_e, z_e]).T
        pose1[:-1, -1] = point0.T
        return pose1

    @staticmethod
    def normalize_vector(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def spherical_to_cartesian(center, r, the, phi):
        x = r * np.sin(the) * np.cos(phi) + center[0]
        y = r * np.sin(the) * np.sin(phi) + center[1]
        z = r * np.cos(the) + center[2]
        return np.array([x, y, z])
