import time
import math
import itertools
import random
import os
import pandas as pd
import numpy as np
import argparse
import matplotlib.pyplot as plt
import utils
import sim


def fit_line(x_points, y_points):
    x_mu = np.mean(x_points)
    y_mu = np.mean(y_points)

    x_diff = x_points - x_mu
    y_diff = y_points - y_mu

    numerator = -2 * np.dot(x_diff, y_diff)
    denominator = np.sum(y_diff**2 - x_diff**2)

    alpha = np.arctan2(numerator, denominator) / 2

    rho = x_mu * np.cos(alpha) + y_mu * np.sin(alpha)

    if rho < 0:
        alpha = alpha + math.pi

        if alpha > math.pi:
            alpha = alpha - 2 * math.pi
        rho = -rho

    return (alpha, rho)


def RANSAC(x_points, y_points, inlier_D, steps):
    assert len(x_points) == len(y_points)

    num_points = len(x_points)
    combinations = list(itertools.combinations(range(num_points), 2))
    if len(combinations) > steps:
        combinations = np.array(random.sample(combinations, steps))

    best_line = None
    best_inliers = []
    best_inlier_count = -1

    for pt_1, pt_2 in combinations:
        x, y = x_points[[pt_1, pt_2]], y_points[[pt_1, pt_2]]
        line = fit_line(x, y)

        line_D = np.abs(
            line[1] - x_points * np.cos(line[0]) - y_points * np.sin(line[0])
        )
        inliers = line_D < inlier_D
        inlier_count = np.count_nonzero(inliers)

        if inlier_count > best_inlier_count:
            best_line = line
            best_inliers = inliers
            best_inlier_count = inlier_count

    return best_line, best_inliers


def iterative_RANSAC(x_points, y_points, min_inliers=10, inlier_D=0.1, steps=50):
    lines = []
    while True:
        line, inliers = RANSAC(x_points, y_points, inlier_D, steps)

        # POINT PLOTTING
        # plt.scatter(x_points, y_points)
        # plt.scatter(x_points[inliers], y_points[inliers], c='green')
        # plt.show()

        if np.count_nonzero(inliers) < min_inliers and len(lines) > 0:
            break
        lines.append(line)
        x_points = x_points[~inliers]
        y_points = y_points[~inliers]
    return lines


def noise_measurement(lines, k1=0.001, k2=0.0001):
    k1 = max(0.001, k1)
    k2 = max(0.0001, k2)

    new_Z = []
    new_R = []

    for line in lines:
        cov = np.eye(2)
        cov[0, 0] *= k2
        cov[1, 1] *= k1
        cov *= np.abs(line[1])

        new_z = np.random.multivariate_normal(line, cov)
        new_Z.append(new_z)
        new_R.append(cov)

    return new_Z, new_R


class CoppeliaSimulation:
    def __init__(self):
        sim.simxFinish(-1)
        self.client_id = sim.simxStart("127.0.0.1", 19999, True, True, 1000, 5)
        assert (
            self.client_id != -1
        ), "Connection to Coppelia failed. Is the simulation running?"

    def _call_func(self, name, ints=[], floats=[], strings=[]):
        """
        Call a function from the robot.
        The functions can be found in the "Funciones" file in the scene file
        """

        return sim.simxCallScriptFunction(
            self.client_id,
            "Funciones",
            sim.sim_scripttype_childscript,
            name,
            ints,
            floats,
            strings,
            bytearray("1", "utf-8"),
            sim.simx_opmode_blocking,
        )

    def _timer(self, wait_time):
        """
        Timer that uses simulation time.
        Input can be float or int
        """
        _, _, start_time, _, _ = self._call_func("CurrentTick")
        start_time = start_time[0]
        while True:
            _, _, current_time, _, _ = self._call_func("CurrentTick")

            if (current_time[0] - start_time) > wait_time:
                break

    def _read_LiDAR(self):
        # The coordinates are relative to the robot!
        raw_data = np.array(self._call_func("SensorLiDAR", [])[2])

        x = raw_data[0::3]
        y = raw_data[1::3]

        return x, y

    def _read_pos(self):
        # The coordinates are absolute (i.e. don't use this, because irl we don't know this)
        x, y, theta = self._call_func("car_pos")[2]
        return x, y, theta

    def get_lines(self):
        # The lines will be relative to the robot
        x, y = self._read_LiDAR()
        lines = iterative_RANSAC(x, y, steps=100, min_inliers=2)

        return lines

    def move(self, speed):
        speed_x = speed[0] / 0.028
        speed_y = speed[1] / 0.028

        self._call_func("On", [1], [speed_x])
        self._call_func("On", [2], [speed_y])
        self._timer(1)
        self._call_func("Off", [3])

    def __del__(self):
        sim.simxFinish(-1)
