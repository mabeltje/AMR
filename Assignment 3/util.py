import numpy as np
from numpy.random import normal
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import math
import random

def newLine(x1, x2, y1, y2, color="green", marker=None, linewidth=3):
    """
    This function creates a line object for matplotlib
    input:
        - x1 : float
        - x2 : float
        - y1 : float
        - y2 : float
    output:
        - l : line opject
    """
    l = mlines.Line2D([x1, x2], [y1, y2], color=color, marker=marker, linewidth=linewidth)
    return l

def newCircle(x, y, radius=0.2, color="blue", marker=None):
    """
    This function creates a circle object for matplotlib
    input:
        - x : float
        - x : float
        - radius : float
    output:
        - c : circle object
    """
    c = mpatches.Circle((x, y), radius=radius, color=color)
    return c
    
def find_intersection(rho_1, alpha_1, rho_2, alpha_2, bounds = [0, 16, 0, 16]):
    """
    This function finds the intersection between to polar lines.
    input:
        - rho_1 : float
        - alpha_1 : float
        - rho_2 : float
        - alpha_2 : float
    output:
        - coords : (float, float)
    """

    if alpha_1 == alpha_2:
        return False
    
    rho_diff = rho_1 - rho_2
    
    x_part = np.cos(alpha_1) - np.cos(alpha_2)
    y_part = np.sin(alpha_1) - np.sin(alpha_2)
    
    
    if alpha_1 == 0:
        y = (rho_diff - (rho_1 * x_part))/(y_part)
        if y > bounds[3] or y < bounds[2]:
            return False
        return (rho_1, y)

    if alpha_1 == math.pi/2:
        x = (rho_diff - (rho_1 * y_part))/(x_part)
        if x > bounds[1] or x < bounds[0]:
            return False
        return (x, rho_1)

def polar_to_cartesian(rho, alpha, bounds = [0, 16, 0, 16]):
    """
    This function convert a polar line to two cartesian points to make a line
    input:
        - rho : float
        - alpha : float
    output:
        - two cartesian points : floats
    """
    points = []

    p = find_intersection(bounds[0], 0, rho, alpha, bounds)
    if p:
        points.append(p)

    p = find_intersection(bounds[1], 0, rho, alpha, bounds)
    if p:
        points.append(p)
    
    p = find_intersection(bounds[2], math.pi/2, rho, alpha, bounds)
    if p:
        points.append(p)
    
    p = find_intersection(bounds[3], math.pi/2, rho, alpha, bounds)
    if p:
        points.append(p)
    
    if len(points) < 2:
        print("This line is out of bounds")
        return False
    return points[0][0], points[1][0], points[0][1], points[1][1]

def points_helper(x1, x2, y1, y2, number, epsilon):
    """
    This function interpolates between two points
    input:
        - x_1 : float
        - x_2 : float
        - y_1 : float
        - y_2 : float
        - number : int
        - epsilon: float
    output:
        - coords : [(float, float)]
    """
    x_steps = (x2 - x1)/number
    y_steps = (y2 - y1)/number
    
    x_res = []
    y_res = []
    for i in range(number):
        x_res.append((x1+x_steps*i) + normal()*epsilon)
        y_res.append((y1+y_steps*i) + normal()*epsilon)
    
    return x_res, y_res

def get_line_points(x, y, number, epsilon):
    """
    This function creates points with noise of a combination of lines
    input:
        - x : [float]
        - y : [float]
        - number : int
        - epsilon: float
    output:
        - coords : [(float, float)]
    """
    x_res = []
    y_res = []
    
    for i in range(len(x)-1):
        x_temp, y_temp = points_helper(x[i], x[i+1], y[i], y[i+1], number, epsilon)
        x_res += x_temp
        y_res += y_temp
    
    return np.array(x_res), np.array(y_res)

def visualize(x_points=[], y_points=[], lines=[]):
    """
    This function visualizes points and lines
    input:
        - x_points : [float]
        - y_points : [float]
        - lines : [(rho, alpha)]
    """
    width, height = 16, 16

    fig, ax = plt.subplots(figsize = (10,10))
    ax.set(xlim=[0, width], ylim=[0, height])

    for x,y in zip(x_points, y_points):
        c = newCircle(x, y)
        ax.add_artist(c)
 
    for rho, alpha in lines:
        x1, x2, y1, y2 = polar_to_cartesian(rho, alpha)
        l = newLine(x1, x2, y1, y2, color="red")
        ax.add_artist(l)
        
    # Be sure to draw the canvas once before we start blitting. Otherwise
    # a) the renderer doesn't exist yet, and b) there's noting to blit onto
    fig.canvas.draw()

    plt.show()


def visualize_room(x_points=[], y_points=[], lines=[], bounds=[-10, 10, -10, 10], room_corners = None):
    """
    This function visualizes points and lines
    input:
        - x_points : [float]
        - y_points : [float]
        - lines : [(rho, alpha)]
    """
    x_start, y_start = bounds[0], bounds[2]
    x_end, y_end = bounds[1], bounds[3]

    fig, ax = plt.subplots(figsize = (10,10))
    ax.set(xlim=[x_start, x_end], ylim=[y_start, y_end])

    if room_corners is not None:
        corners_no = len(room_corners)
        for i in range(corners_no):
            x1, y1 = room_corners[i % corners_no]
            x2, y2 = room_corners[(i + 1) % corners_no]
            l = newLine(x1, x2, y1, y2, color="green", linewidth=3)
            ax.add_artist(l) 

    for x,y in zip(x_points, y_points):
        c = newCircle(x, y, radius=2.5)
        ax.add_artist(c)
 
    for rho, alpha in lines:
        x1, x2, y1, y2 = polar_to_cartesian(rho, alpha, 
                                            bounds=[x_start, x_end, y_start, y_end])
        l = newLine(x1, x2, y1, y2, color="red", linewidth=2)
        ax.add_artist(l)
        
    # Be sure to draw the canvas once before we start blitting. Otherwise
    # a) the renderer doesn't exist yet, and b) there's noting to blit onto
    fig.canvas.draw()

    plt.show()


def visualize_furthest(x_points, y_points, x_furthest, y_furthest, line):
    """
    This function visualizes a line, all points and the furthest point
    input:
        - x_points : [float]
        - y_points : [float]
        - x_furthest : float
        - y_furthest : float
        - lines : [(rho, alpha)]
    """
    width, height = 16, 16

    fig, ax = plt.subplots(figsize = (10,10))
    ax.set(xlim=[0, width], ylim=[0, height])

    for x,y in zip(x_points, y_points):
        c = newCircle(x, y, color="blue")
        ax.add_artist(c)
        
    c = newCircle(x_furthest, y_furthest, color="green")
    ax.add_artist(c)
 
    x1, x2, y1, y2 = polar_to_cartesian(line[0], line[1])
    l = newLine(x1, x2, y1, y2, color="red")
    ax.add_artist(l)
        
    # Be sure to draw the canvas once before we start blitting. Otherwise
    # a) the renderer doesn't exist yet, and b) there's noting to blit onto
    fig.canvas.draw()

    plt.show()

def visualize_split(x_points_1, y_points_1, x_points_2, y_points_2, line):
    """
    This function visualizes a line and the split of points 
    input:
        - x_points_1 : [float]
        - y_points_1 : [float]
        - x_points_2 : [float]
        - y_points_2 : [float]
        - lines : [(rho, alpha)]
    """
    width, height = 16, 16

    fig, ax = plt.subplots(figsize = (10,10))
    ax.set(xlim=[0, width], ylim=[0, height])

    for x,y in zip(x_points_1, y_points_1):
        c = newCircle(x, y, color="blue")
        ax.add_artist(c)
        
    for x,y in zip(x_points_2, y_points_2):
        c = newCircle(x, y, color="green")
        ax.add_artist(c)
 
    x1, x2, y1, y2 = polar_to_cartesian(line[0], line[1])
    l = newLine(x1, x2, y1, y2, color="red")
    ax.add_artist(l)
        
    # Be sure to draw the canvas once before we start blitting. Otherwise
    # a) the renderer doesn't exist yet, and b) there's noting to blit onto
    fig.canvas.draw()

    plt.show()


def get_coords(exercise):
    """
    This function returns points used in the exercises
    input:
        - exercise : int
    output:
        - coords : [float]
    """
    if exercise == 1:
        x_points = np.array([0.04448639, 1.75890721, 3.39092071, 
                     5.03992991, 6.56323536, 8.16756932])

        y_points = np.array([9.98438863, 10.04832092,  9.85492495,  
                            9.93781162,  9.93272615, 10.13196551])

    if exercise == 2:
        x_points = np.array([10, 10, 10, 10, 10, 10, 10, 10, 8.57142857, 7.14285714, 5.71428571, 4.28571429, 2.85714286,  1.42857143]) 
        y_points = np.array([0,  1.42857143,  2.85714286,  4.28571429,  5.71428571,  7.14285714, 8.57142857, 10, 10, 10, 10, 10, 10, 10])
         


    if exercise == 3:
        L1_x = np.array([6.56050522, 5.03274245, 0.07085426, 8.17608616, 9.95440092,
        1.76112919, 3.3038893 ])
        L1_y = np.array([ 9.84145918,  9.96393957,  9.80545067,  9.89037595, 10.00203053,
                10.09557847,  9.97688341])
        L2_x = np.array([10.10366057,  9.98151464,  9.95440092, 10.04445705,  9.8940228 ,
                10.12058128])
        L2_y = np.array([ 6.68359018,  5.07589423, 10.00203053,  3.30031845,  8.34526453,
                1.62103725])

        return L1_x, L1_y, L2_x, L2_y    

        
    return x_points, y_points

def get_arg():
    return 7

def get_line(exercise):
    """
    This function returns lines used in the exercises
    input:
        - exercise : int
    output:
        - rho : float
        - alpha : float
    """
    if exercise == 1:
        return 10, math.pi/2
    if exercise == 2:
        return 7, math.pi/4
    if exercise == 3:
        return [(10, 0), (10, math.pi/2)]


def add_noise(points, epsilon):
    """
    This function adds noise to a list of numbers
    input:
        - points : float
        - epsilon : float
    output:
        - points : float
    """
    return np.array([p + normal()*epsilon for p in points])

def get_single_line(epsilon=0.1):
    """
    This function returns points on a single line with some randomness
    input:
        - epsilon : float
    output:
        - coords : [float]
    """
    r = 1

    x_points = add_noise([10, 0], r)
    y_points = add_noise([0, 10], r)

    x_points[x_points<0] = 0
    y_points[y_points<0] = 0

    x_points[x_points>16] = 0
    y_points[y_points>16] = 0
    
    return get_line_points(x_points, y_points, 10, epsilon) 

def get_double_line(epsilon=0.1):
    """
    This function returns points on a double line with some randomness
    input:
        - epsilon : float
    output:
        - coords : [float]
    """
    r = 0.7

    x_points = add_noise([10, 10, 0], r)
    y_points = add_noise([0, 10, 10], r)

    x_points[x_points<0] = 0
    y_points[y_points<0] = 0

    x_points[x_points>16] = 0
    y_points[y_points>16] = 0

    return get_line_points(x_points, y_points, 7, epsilon) 

def get_triple_line(epsilon=0.1):
    """
    This function returns points on a triple line with some randomness
    input:
        - epsilon : float
    output:
        - coords : [float]
    """
    r = 0.3

    x_points = add_noise([10, 10, 6, 0], r)
    y_points = add_noise([0, 6, 10, 10], r)

    x_points[x_points<0] = 0
    y_points[y_points<0] = 0

    x_points[x_points>16] = 0
    y_points[y_points>16] = 0

    return get_line_points(x_points, y_points, 5, epsilon) 

def get_room_corners():
    return [(-98, 98), (-98, -95), (192, -95), (192, 98)]