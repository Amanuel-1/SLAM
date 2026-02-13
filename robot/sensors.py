"""Sensor models for robot"""
from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np
from utils.noise import add_gaussian_noise
from utils.geometry import distance,lerp,draw_line

import config
import math

class Sensor(ABC):
    """Base sensor class"""
    
    def __init__(self, noise_std: float = 0.0):
        self.noise_std = noise_std
    
    @abstractmethod
    def sense(self, robot, world):
        """Get sensor reading"""
        pass


class Lidar(Sensor):
    """2D Lidar sensor"""
    
    def __init__(self, world_map, robot=None, num_beams: int = config.LIDAR_BEAMS, 
                 max_range: float = config.LIDAR_RANGE,
                 fov: float = config.LIDAR_FOV,
                 noise_std: float = config.LIDAR_NOISE_SIGMA):
        super().__init__(noise_std)
        self.num_beams = num_beams
        self.max_range = max_range
        self.fov = np.radians(fov)
        self.robot = robot
        self.pos: Tuple = (0, 0)
        self.map = world_map
    
    def sense(self) -> List[Tuple]:
        """Get lidar range readings"""
        if self.robot:
            self.pos = (self.robot.pose.x, self.robot.pose.y)
            robot_theta = self.robot.pose.theta
        else:
            robot_theta = 0
        
        output = []
        
        map_width = self.map.get_width()
        map_height = self.map.get_height()
        
        (xi, yi) = self.pos
        
        for theta in np.linspace(0, config.LIDAR_FOV, self.num_beams):
            world_theta = theta + robot_theta
            
            xj = xi + (self.max_range * math.cos(world_theta))
            yj = yi + (self.max_range * math.sin(world_theta))

            for i in range(200):
                u = i / 200
                x = int(lerp(xi, xj, u))
                y = int(lerp(yi, yj, u))

                if x < 0 or x >= map_width or y < 0 or y >= map_height:
                    break

                if self.map.get_at((x, y)) == (0, 0, 0):
                    dist = distance((x, y), self.pos)

                    if config.LIDAR_ENABLE_NOISE:
                        dist, angle = add_gaussian_noise(dist, world_theta, 
                                                         config.LIDAR_NOISE_SIGMA, 
                                                         enable_noise=True)
                    else:
                        angle = world_theta

                    output.append((dist, angle, self.pos))
                    break
        
        return output
    
    # def _raycast(self, x: float, y: float, angle: float, world_map) -> float:
    #     """Cast a ray and return distance to obstacle"""
    #     # Simple raycasting implementation
    #     step_size = world_map.resolution
    #     distance = 0.0
        
       
    #     return self.max_range

