################################################################################
#
# Student Names: Jeroen Bannenberg, Mabel Traube
#
#
# Student Numbers: 13153900, 13982656
#
#
# Group number: 4
#
#
###############################################################################

try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

sim.simxFinish(-1)  # just in case, close all opened connections

port = 19999  # change to 19999 for windows/linux
clientID = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)  # Connect to CoppeliaSim

import time
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


def timer(wait_time):
    """
        Timer that uses simulation time.
        Input can be flaots or ints
    """
    _, _, start_time, _, _ = call_func("CurrentTick")
    start_time = start_time[0]
    while True:
        _, _, current_time, _, _ = call_func("CurrentTick")

        if (current_time[0] - start_time) > wait_time:
            break


def exercise_1():
    '''
        Exercise 1: drive the car through the maze without using sensors
    '''

    call_func('On', [3, 19])
    timer(2.5)
    call_func('Off', [3])
    call_func('On', [1, 19])
    timer(0.17)
    call_func('Off', [3])
    call_func('On', [3, 19])
    timer(2.7)
    call_func('Off', [3])
    call_func('On', [2, 19])
    timer(0.18)
    call_func('Off', [3])
    call_func('On', [3, 19])
    timer(4)
    call_func('Off', [3])

    return


def stop_wall():
    '''
        Keep driving until the robot is close to a wall, then stop.
    '''
    call_func('On', [3, 19])

    while (True):
        distance = call_func('SensorSonar')

        if distance[2][0] < 25:
            break

    call_func('Off', [3])
    return


def check_sides():
    '''
        Check the right side of the robot, and if there is a wall, turn left.
    '''

    timer(1)

    # Keep turning until the robot has turned 90 degrees
    call_func('On', [1, 2])
    call_func('On', [2, -2])

    while True:
        if call_func('SensorGyroA')[1][0] >= 90:
            break

    # Stop turning and check the distance to the wall
    call_func('Off', [3])
    distance = call_func('SensorSonar')

    # If the distance is less than 25, turn left
    if distance[2][0] < 25:
        call_func('On', [1, -2])
        call_func('On', [2, 2])

        while True:
            if call_func('SensorGyroA')[1][0] <= -90:
                break

    call_func('Off', [3])
    call_func('ResetGyroA')

    timer(1)
    return


def exercise_2():
    '''
        Exercise 2: drive the car through the maze using sensors
    '''

    stop_wall()
    check_sides()
    stop_wall()
    check_sides()
    stop_wall()

    return


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--exercise', type=int, default=1,
                        help='State what exercise needs to be run')
    ARGS = parser.parse_args()

    if clientID != -1:
        print('Connected to remote API server')
        if ARGS.exercise == 1:
            print("execute exercise 1")
            exercise_1()

        elif ARGS.exercise == 2:
            print("execute exercise 2")
            exercise_2()

        else:
            print("No exercise executed")

    else:
        print('Failed connecting to remote API server')

    sim.simxFinish(-1)
