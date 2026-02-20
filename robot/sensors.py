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
    
    def __init__(self, world_map,num_beams: int = config.LIDAR_BEAMS, 
                 max_range: float = config.LIDAR_RANGE,
                 fov: float = config.LIDAR_FOV,
                 noise_std: float = config.LIDAR_NOISE_SIGMA,
                 ):

        super().__init__(noise_std)
        self.num_beams = num_beams
        self.max_range = max_range
        self.fov = np.radians(fov)
        self.pos:Tuple = (0,0)
        self.map = world_map
    
    def sense(self) -> List[Tuple]:
        """Get lidar range readings"""
        output = []
        
        # Get map dimensions
        map_width = self.map.get_width()
        map_height = self.map.get_height()
        
        (xi, yi) = self.pos
        # Use the configured number of beams instead of hardcoded 50
        for theta in np.linspace(0, config.LIDAR_FOV, self.num_beams):
           
            xj = xi + (self.max_range * math.cos(theta))
            yj = yi + (self.max_range * math.sin(theta))

            # draw_line(self.map,self.pos,(xj,yj))
            # Increase ray resolution for better accuracy (200 steps instead of 100)
            for i in range(200):
                u = i / 200
                x = int(lerp(xi, xj, u))
                y = int(lerp(yi, yj, u))

                # Check bounds
                if x < 0 or x >= map_width or y < 0 or y >= map_height:
                    break

                if self.map.get_at((x,y))==(0,0,0):
                    """
                    calculate the distance from this point to the sensor's position
                    add the noise to the distance and angle 
                    then return the output sensor
                    the sensor data should have this structure
                    Output: (angle,distance,sensor_position)
                    """ 
                    dist = distance((x,y),self.pos)

                    # Apply noise if enabled
                    if config.LIDAR_ENABLE_NOISE:
                        dist, angle = add_gaussian_noise(dist, theta, 
                                                         config.LIDAR_NOISE_SIGMA, 
                                                         enable_noise=True)
                    else:
                        angle = theta

                    output.append((dist, angle, self.pos))
                    break

        
        return output
    
    # def _raycast(self, x: float, y: float, angle: float, world_map) -> float:
    #     """Cast a ray and return distance to obstacle"""
    #     # Simple raycasting implementation
    #     step_size = world_map.resolution
    #     distance = 0.0
        
       
    #     return self.max_range

