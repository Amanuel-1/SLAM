"""Geometric transformation utilities"""
import numpy as np
import math
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
    x = xi + x* math.cos(angle)
    y = -yi + y* math.sin(angle)

    return (x,y)
