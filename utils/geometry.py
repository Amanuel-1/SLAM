"""Geometric transformation utilities"""
import numpy as np
import math
import pygame
from typing import Tuple
from scipy.odr import odr, Model
from fractions import Fraction


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

def transform_polar_point_set(data, min_distance=1.0):
    """
    Transform polar lidar data to Cartesian coordinates, filter duplicates, and sort by angle.
    
    The seeded region growing algorithm requires points to be ordered by scan angle
    so that consecutive points in the array correspond to consecutive scan angles.
    
    Args:
        data: List of tuples (distance, angle, position)
        min_distance: Minimum distance between consecutive points to avoid duplicates (pixels)
    
    Returns:
        List of tuples ((x, y), angle) sorted by angle, with duplicates filtered
    """
    point_coordinates = []
    if data:
        for (distance, angle, position) in data:
            coordinate = polar2p(distance, angle, position)
            point_coordinates.append((coordinate, angle))
        
        # Sort by angle to ensure proper ordering for line extraction
        # This is critical for the seeded region growing algorithm
        point_coordinates.sort(key=lambda p: p[1])  # Sort by angle (second element)
        
        # Filter out points that are too close together (duplicates or near-duplicates)
        # Also filter based on angular separation to reduce noise effects
        if len(point_coordinates) > 1:
            filtered_points = [point_coordinates[0]]  # Always keep first point
            min_angular_separation = 0.001  # Minimum angular separation in radians (~0.06 degrees)
            
            for i in range(1, len(point_coordinates)):
                prev_point, prev_angle = filtered_points[-1]
                curr_point, curr_angle = point_coordinates[i]
                
                # Calculate spatial and angular separation
                spatial_dist = point_point_distance(prev_point, curr_point)
                angular_sep = abs(curr_angle - prev_angle)
                
                # Keep point if it's far enough spatially AND angularly
                # This helps filter out noise-induced duplicates
                if spatial_dist >= min_distance or angular_sep >= min_angular_separation:
                    filtered_points.append(point_coordinates[i])
            
            point_coordinates = filtered_points
    
    return point_coordinates

def draw_line(surface,p1,p2):
    xi,yi = p1
    x2,y2 = p2

    pygame.draw.line(surface,(255,0,0),p1,p2)


def point_line_distance(point, line_params):
    """
        - this function returns a distance between a point and a line given in general form
        - Line equation: Ax + By + C = 0
        - Distance formula: |Ax + By + C| / sqrt(A^2 + B^2)
    """
    A, B, C = line_params
    x, y = point
    distance = abs(A*x + B*y + C) / math.sqrt(A**2 + B**2)
    
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

def points_to_line(point1, point2):
    """
    Compute slope and y-intercept of line through two points.
    
    Args:
        point1: First point (x1, y1)
        point2: Second point (x2, y2)
    
    Returns:
        (m, b) where y = m*x + b, or None if line is vertical
    """
    x1, y1 = point1
    x2, y2 = point2
    
    # Check if line is vertical (use tolerance for floating point comparison)
    if abs(x1 - x2) < 1e-10:
        # The line is vertical and the slope is infinite
        return None
    
    m = (y2 - y1) / (x2 - x1)
    b = y1 - (m * x1)
    return m, b
def project_point_to_line(point, m, b):
    x, y = point

    # x projection
    x_projection = (x + m*(y - b)) / (1 + m**2)

    # y projection (must add + b)
    y_projection = m * x_projection + b

    return x_projection, y_projection

def line_func(B, x):
    """
    Linear model function for scipy.odr.
    
    Args:
        B: Parameter vector [m, b] where m is slope and b is y-intercept
        x: Independent variable (x-coordinate)
    
    Returns:
        y = m*x + b
    """
    m, b = B
    return m*x + b

from scipy.odr import Model, RealData, ODR
import numpy as np
from fractions import Fraction

def odr_fit(points):
    """
    Orthogonal Distance Regression (ODR) for fitting a line to points.
    Returns slope (m) and y-intercept (b) of the fitted line.
    
    Args:
        points: List of tuples, where each tuple is ((x, y), angle) or (x, y)
    
    Returns:
        (m, b): slope and y-intercept of the fitted line
    """
    # Extract x and y coordinates
    # Handle both formats: ((x, y), angle) or (x, y)
    if isinstance(points[0], tuple) and len(points[0]) == 2:
        if isinstance(points[0][0], tuple):
            # Format: ((x, y), angle)
            x = np.array([p[0][0] for p in points])
            y = np.array([p[0][1] for p in points])
        else:
            # Format: (x, y)
            x = np.array([p[0] for p in points])
            y = np.array([p[1] for p in points])
    else:
        raise ValueError("Invalid point format")

    # Define the linear model
    linear_model = Model(line_func)

    # Create RealData object
    data = RealData(x, y)
    
    # Initial guess for parameters [m, b]
    beta0 = [0., 0.]
    
    # Set up orthogonal distance regression 
    odr_instance = ODR(data, linear_model, beta0=beta0)
    
    # Run the regression
    fitted_line = odr_instance.run()

    # Extract slope and intercept
    m, b = fitted_line.beta

    return m, b
def get_predicted_point(line_params, sensed_point, position):
    """
    Returns a predicted point by intersecting the fitted line and the
    line from the robot's position to the sensed point (the laser beam).
    
    Args:
        line_params: Line parameters in general form (A, B, C) where Ax + By + C = 0
        sensed_point: The sensed point (x, y)
        position: Sensor position (x0, y0)
    
    Returns:
        (x, y) intersection point, or None if lines are parallel
    """
    x, y = sensed_point
    x0, y0 = position
    
    # Handle vertical laser beam case (x0 == x)
    if abs(x0 - x) < 1e-10:
        # Vertical line: x = x0
        # Find intersection with fitted line: Ax + By + C = 0
        A, B, C = line_params
        if abs(B) < 1e-10:
            # Fitted line is also vertical, no intersection
            return None
        # Substitute x = x0 into fitted line equation
        y_intersect = -(A * x0 + C) / B
        return (x0, y_intersect)
    
    # Get the line for the laser beam
    line_result = points_to_line((x0, y0), (x, y))
    if line_result is None:
        return None
    
    m2, b2 = line_result
    laser_line_params = slope_intersept_to_general(m2, b2)
    
    # Get the intersection between the two lines
    intersection = intersection_point_general(line_params, laser_line_params)
    return intersection

