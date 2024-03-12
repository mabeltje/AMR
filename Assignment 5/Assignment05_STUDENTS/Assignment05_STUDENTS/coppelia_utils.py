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

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


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
            alpha = alpha - 2*math.pi
        rho = -rho

    return (alpha, rho)


def RANSAC(x_points, y_points,
                  inlier_D,
                  steps):
    
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

        line_D = np.abs(line[1] - x_points * np.cos(line[0]) - y_points * np.sin(line[0]))
        inliers = line_D < inlier_D
        inlier_count = np.count_nonzero(inliers)

        if inlier_count > best_inlier_count:
            best_line = line
            best_inliers = inliers
            best_inlier_count = inlier_count

    return best_line, best_inliers


def iterative_RANSAC(x_points, y_points,
                     min_inliers = 10,
                     inlier_D = 0.1,
                     steps = 50):
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


def noise_measurement(Z, k1= 0.001, k2=0.0001):
    k1 = max(0.001, k1)
    k2 = max(0.0001, k2)

    new_Z = []
    new_R = []

    for line in Z:
        cov = np.eye(2)
        cov[0,0] *= k2
        cov[1,1] *= k1
        cov *= np.abs(line[1])

        new_z = np.random.multivariate_normal(line, cov)
        new_Z.append(new_z)
        new_R.append(cov)

    return new_Z, new_R


def get_environment_map(env_id):
    if env_id == 1:
        return [
            (0, 1),
            (np.pi / 2, 1),
            (-np.pi / 2, 1),
            (np.pi, 2),
            (-3/4 * np.pi, 2**0.5)
        ]
    raise ValueError("Select proper environment id")


class SimulationRun():

    def __init__(self):
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

    def call_func(self, name, ints=[], floats=[], strings=[]):
        '''
            Call a function from the robot.
            The functions can be found in the "functiones" file in the scene file
        '''

        return sim.simxCallScriptFunction(self.clientID, 'Funciones', 
                                        sim.sim_scripttype_childscript, 
                                        name, ints, floats, strings, 
                                        bytearray('1', 'utf-8'), 
                                        sim.simx_opmode_blocking)
    
    def timer(self, wait_time):
        """
            Timer that uses simulation time.
            Input can be flaots or ints
        """
        _, _, start_time, _, _ = self.call_func("CurrentTick")
        start_time = start_time[0]
        while True:
            _,_, current_time, _, _ = self.call_func("CurrentTick")

            if (current_time[0] - start_time) > wait_time:
                break 

    def read_LiDAR(self):
            raw_data = np.array(self.call_func("SensorLiDAR", [])[2])

            x = raw_data[0::3]
            y = raw_data[1::3]
            z = raw_data[2::3]

            return x, y

    def run_simulation(self, transition_function, kalman_step, M,
                       k: float = 0.1, g: float = 0.5, iter_steps = 20,
                       speed_mean = [1.2, 1.2],
                       measurement_noise = [0.001, 0.0001]):
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

        # lines = iterative_RANSAC(x, y)
        # print(lines)

        b = 0.1 
        k = k
        g = g
        wheel_mult = 0.0005

        k1, k2 = measurement_noise[0], measurement_noise[1]

        speed_mean = speed_mean
        speed_cov = utils.compute_Q(speed_mean, k)

        x_true = np.array(self.call_func("car_pos")[2])
        x_ol = x_true
        x_kalman = x_true

        true_path = [x_true]
        ol_path = [x_ol]
        kalman_path = [x_kalman]

        P_0 = np.eye(3) * 0.005
        P_ol = P_0
        P_kalman = P_0

        prev_u_1 = 0
        prev_u_2 = 0

        iter = 0

        while iter < iter_steps:
            wheel_speed =  np.random.multivariate_normal(speed_mean, speed_cov)
            
            self.call_func("On", [1], [wheel_speed[0]])
            self.call_func("On", [2], [wheel_speed[1]])

            self.timer(1)

            x_true = np.array(self.call_func("car_pos")[2])
            true_path.append(x_true)

            u_1 = self.call_func("MotorRotationCountB")[1][0]
            u_2 = self.call_func("MotorRotationCountC")[1][0] 

            u = np.mean([u_1 - prev_u_1, u_2 - prev_u_2]) * wheel_mult
            u = np.array([u, u])
            prev_u_1, prev_u_2 = u_1, u_2
            Q = utils.compute_Q(u, k)

            x_ol, F_x, F_u = transition_function(x_ol, u, b)
            ol_path.append(x_ol)
            P_ol = F_x @ P_ol @ F_x.T + F_u @ Q @ F_u.T
            
            x_points, y_points = self.read_LiDAR()
            lines = iterative_RANSAC(x_points, y_points, steps=100, min_inliers=7)
            Z, R = noise_measurement(lines, k1, k2)

            x_kalman, P_kalman = kalman_step(x_kalman, P_kalman, u, k, Z, R, M, g, b)

            kalman_path.append(x_kalman)

            iter += 1

        self.call_func("Off", [3])

        sim.simxFinish(-1)

        return true_path, ol_path, kalman_path