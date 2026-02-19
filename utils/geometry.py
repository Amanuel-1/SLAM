"""Geometric transformation utilities"""
import numpy as np
import math
import pygame
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def world_to_robot_frame(point: Tuple[float, float], 
                         robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
    """Transform point from world frame to robot frame"""
    x, y = point
    rx, ry, theta = robot_pose
    
    dx = x - rx
    dy = y - ry
    
    cos_theta = np.cos(-theta)
    sin_theta = np.sin(-theta)
    
    robot_x = cos_theta * dx - sin_theta * dy
    robot_y = sin_theta * dx + cos_theta * dy
    
    return robot_x, robot_y


def robot_to_world_frame(point: Tuple[float, float], 
                        robot_pose: Tuple[float, float, float]) -> Tuple[float, float]:
    """Transform point from robot frame to world frame"""
    x, y = point
    rx, ry, theta = robot_pose
    
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    world_x = cos_theta * x - sin_theta * y + rx
    world_y = sin_theta * x + cos_theta * y + ry
    
    return world_x, world_y


def distance(p1:Tuple,p2:Tuple):
    p1x,p1y = p1
    p2x,p2y = p2

    return math.sqrt((p2x-p1x)**2 + (p2y-p1y)**2)

def lerp(a,b,t):
    return a *(1-t) + (b*t)

def polar2p(distance:float, angle:float,position:Tuple):
    xi,yi = position
    x = xi + distance * math.cos(angle)
    y = yi + distance * math.sin(angle)

    return (int(x),int(y))

def transform_polar_point_set(data):
    point_coordinates =[]
    if data:
        for (distance, angle,position) in data:
            coordinate = polar2p(distance, angle, positon)
            point_coordinates.append((coordinate,angle))

        #TODO: set the NP variable to len(point_coordinates)-1 
        # NP is the number of points (used in the three sections of the split and merge algorithm)
    
    return point_coordinates

def draw_line(surface,p1,p2):
    xi,yi = p1
    x2,y2 = p2

    pygame.draw.line(surface,(255,0,0),p1,p2)


def point_line_distance(point,line_params):
    """
        - this function returns a distance between a point and a line given in general form
    """
    A,B,C = line_params
    x,y = point
    distance = abs(A*x + C*y +C)/math.sqrt(A**2 + B**2)
    
    return distance
def point_point_distance(point1,point2):
    x1,y1 = point1
    x2,y2 = point2

    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def project_point_on_line():
    """
    Projects a single point onto the line
    """
    "pass"


def get_line_points(m,b):
    """
    we use this function to get two points from a line given in slope equation
    """
    #randomly select two points to get their image on the line
    x0,x1 = 5, 2000

    y0 = m*x0 + b
    y1 = m*x1 + b

    return ((x0,y0), (x1,y1))

def general_to_slope_intercept(A, B, C):
    """
    This function converts line equation from general form to intercept form
    Ax + By + C = 0
    """
    #handle special cases where the line is verticall , infinite slope
    if B == 0:
        raise ValueError("Line is vertical, no y-intercept")

    m = -A/B
    b = -C/B

    return m, b
 
def slope_intersept_to_general(m,b):
    """
    converts from slope intercept form to general form of a line
    """
    

    # since mx-y+B = 0 is the same as the general form
    A = m
    B = -1
    C = b

    #
    if A < 0:
        A, B, C = -A, -B, -C

    
    A_frac = Fraction(A).limit_denominator(1000)
    C_frac = Fraction(C).limit_denominator(1000)

    den_a = A_frac.denominator
    den_c = C_frac.denominator

    # Step 4: Compute LCM of denominators
    gcd = np.gcd(den_a, den_c)
    lcm = den_a * den_c // gcd

    # Step 5: Clear denominators
    A_int = int(A_frac * lcm)
    B_int = int(B * lcm)
    C_int = int(C_frac * lcm)

    return A_int, B_int, C_int


def intersection_point_general(params1,params2):
    A1,B1,C1 = params1
    A2,B2,C2 = params2
    determinant =A1*B2 - A2*B1

    if determinant == 0:
        #if th lines are parallel or are the same return nothing
        return None
    
    x = (B1*C2 - B2*C1)/determinant
    y = (A2*C1 - A1*C2)/determinant

    return x,y

def points_to_line(point1,point2):
    x1,y1 = point1
    x2,y2 = point2 
    if x1 == x2:
        #the line is vertival and the slope is infinite
        return None
    m = (y2-y1)/(x2-x1)
    b = y1-(m*x1)
    return m,b
def project_point_to_line(point, m, b):
    x, y = point

    # x projection
    x_projection = (x + m*(y - b)) / (1 + m**2)

    # y projection (must add + b)
    y_projection = m * x_projection + b

    return x_projection, y_projection




