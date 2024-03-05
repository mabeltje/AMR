################################################################################
#
# Student Names:
#
#
# Student Numbers:
#
#
# Group number:
#
#
###############################################################################

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

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

import time
import os
import pandas as pd
import numpy as np
import argparse

def call_func(name, ints=[], floats=[], strings=[]):
    '''
        Call a function from the robot.
        The functions can be found in the "functiones" file in the scene file
    '''

    return sim.simxCallScriptFunction(clientID, 'Funciones',
                                      sim.sim_scripttype_childscript,
                                      name, ints, floats, strings,
                                      bytearray('1', 'utf-8'),
                                      sim.simx_opmode_blocking)


def read_LiDAR():
    raw_data = np.array(call_func("SensorLiDAR", [])[2])
    x = raw_data[2::4]
    y = raw_data[0::4]
    d = raw_data[3::4]

    xs = np.array_split(x, 4)
    ys = np.array_split(y, 4)
    ds = np.array_split(d, 4)

    rotation_angles = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

    for i in range(4):
        theta = rotation_angles[i]
        v = np.array([xs[i], ys[i]])
        d = ds[i]

        transformed_v = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ]) @ v

        x = transformed_v[0, :][d < 90]
        y = transformed_v[1, :][d < 90]

        xs[i] = x
        ys[i] = y

    x_points = np.concatenate(xs)
    y_points = np.concatenate(ys)

    return x_points * 100, y_points * 100


def timer(wait_time):
    """
        Timer that uses simulation time.
        Input can be flaots or ints
    """
    _, _, start_time, _, _ = call_func("CurrentTick")
    start_time = start_time[0]
    while True:
        _,_, current_time, _, _ = call_func("CurrentTick")

        if (current_time[0] - start_time) > wait_time:
            break


def exercise_sonar():
    '''
        Exercise 3.1: gather measurements (in cartesian coordinate system) of points on the walls of the room using sonar sensor of the robot.
    '''
    x_points, y_points = [], []
    results = {}

    call_func('On', [1, 1])
    call_func('On', [2, -1])

    # Do a full rotation and measure the distance to the walls and save the results
    while call_func('SensorGyroA')[1][0] < 360:
        distance = call_func('SensorSonar')
        angle = call_func('SensorGyroA')[1][0]

        # No wall detected if distance is 255.0
        if distance[2][0] == 255.0:
            continue

        results[angle] = distance[2][0]

    call_func('Off', [3])

    # Convert the polar coordinates to cartesian coordinates
    for angle, distance in results.items():
        x_points.append(distance * np.cos(np.radians(angle)))
        y_points.append(distance * np.sin(np.radians(angle)))

    coords_df = pd.DataFrame({'x': x_points, 'y': y_points})
    coords_df.to_csv('measurements/sonar_coords.csv')
    return


def exercise_lidar():
    '''
        Gather measurements (in cartesian coordinate system) of points on the walls of the room using LiDAR sensor of the robot.
    '''
    x_points, y_points = read_LiDAR()

    coords_df = pd.DataFrame({'x': x_points, 'y': y_points})
    coords_df.to_csv('measurements/lidar_coords.csv')
    return


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--exercise', type=str, default="sonar",
                        help='State what exercise needs to be run')
    ARGS = parser.parse_args()

    os.makedirs('measurements', exist_ok=True)

    if clientID!=-1:
        print ('Connected to remote API server')
        if ARGS.exercise == "sonar":
            print("execute exercise with sonar sensor")
            exercise_sonar()

        elif ARGS.exercise == "lidar":
            print("execute exercise with LiDAR sensor")
            exercise_lidar()

        else:
            print("No exercise executed")

    else:
        print ('Failed connecting to remote API server')

    sim.simxFinish(-1)