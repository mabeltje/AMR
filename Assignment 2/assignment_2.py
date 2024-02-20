################################################################################
#
# Student Names: Mabel Traube
#
#
# Student Numbers: 13982656
#
#
# Group number: 4
#
#
###############################################################################

try:
    import argparse
    import time
    import sim
    import numpy as np
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

sim.simxFinish(-1)  # just in case, close all opened connections

# change to 19997 if using macbook with a M1/M2/M3 chip, otherwise use 19999
port = 19999
clientID = sim.simxStart('127.0.0.1', port, True, True,
                         5000, 5)  # Connect to CoppeliaSim


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


def differentialInvKin(Xi):
    """Determines the wheel speed need to get the desired direction in the robot frame.

    Args:
        Xi (list[float]): The disered direction of the robot in the robot frame
                          given in terms of x, y and theta.

    Returns:
        list[float]: The wheel speeds necessary to reach desired direction.
    """

    R = [[250/7, 0, 25/14], [250/7, 0, -25/14], [0, 1, 0]]

    return np.dot(R, Xi)

def drive_circle(percentage, radius, time=10):
    """Determines the Xi vector needed to drive a specific circle segment
    and uses the differentialInvKin function to compute the wheel speeds.

    Args:
        percentage (float): The percentage of the circle to drive.
        radius (float): The radius of the circle.
        time (float): The time the robot has to drive the circle.

    Returns:
        list[float]: The wheel speeds necessary to drive desired circle segment.
    """

    distance = 2 * np.pi * radius * percentage
    angle = 2 * np.pi * percentage

    Xi = [distance / time, 0, angle / time]

    # Calculate the wheel speeds
    return differentialInvKin(Xi)

def main():
    '''
        Exercise 3: drive the robot through the maze by using kinematics
    '''

    # Drive straight
    wheel_speeds = drive_circle(1/10000, 1000)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(13)
    call_func('Off', [3])

    # Turn right 45 degrees
    wheel_speeds = drive_circle(1/8, 0.5)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(10)
    call_func('Off', [3])

    # Drive straight
    wheel_speeds = drive_circle(1/10000, 1000)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(4)
    call_func('Off', [3])

    # Turn left 90 degrees
    wheel_speeds = drive_circle(1/4, 0.45)
    call_func('On', [1], [wheel_speeds[1]])
    call_func('On', [2], [wheel_speeds[0]])
    timer(10)
    call_func('Off', [3])

    # Drive straight
    wheel_speeds = drive_circle(1/10000, 1000)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(3)
    call_func('Off', [3])

    # Turn right 45 degrees
    wheel_speeds = drive_circle(140/360, 0.35)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(10)
    call_func('Off', [3])

    # Drive straight
    wheel_speeds = drive_circle(1/10000, 1000)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(11)
    call_func('Off', [3])

    # Turn left 90 degrees
    wheel_speeds = drive_circle(1/4, 0.4)
    call_func('On', [1], [wheel_speeds[1]])
    call_func('On', [2], [wheel_speeds[0]])
    timer(12)
    call_func('Off', [3])

    # Drive straight
    wheel_speeds = drive_circle(1/10000, 1000)
    call_func('On', [1], [wheel_speeds[0]])
    call_func('On', [2], [wheel_speeds[1]])
    timer(12)
    call_func('Off', [3])

    return


if __name__ == "__main__":
    if clientID != -1:
        print('Connected to remote API server')
        print("execute main")
        main()

    else:
        print('Failed connecting to remote API server')

    sim.simxFinish(-1)
