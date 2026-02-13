"""Robot model with pose and motion"""
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import numpy as np
import math
from utils.geometry import normalize_angle, point_point_distance
from utils.noise import add_motion_noise
import config


@dataclass
class Pose:
    """Robot pose in 2D"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)


class Robot:
    """Robot with differential drive motion model"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.pose = Pose(x, y, theta)
        self.true_pose = Pose(x, y, theta)
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.radius = config.ROBOT_RADIUS
        self.sensors: List = []
        self.path: List[Tuple[float, float, float]] = []
        self.path_sample_distance = config.PATH_SAMPLE_DISTANCE
    
    def add_sensor(self, sensor) -> None:
        """Add a sensor to the robot"""
        self.sensors.append(sensor)
    
    def set_velocity(self, v: float, omega: float) -> None:
        """Set robot velocity commands"""
        self.velocity = np.clip(v, -config.MAX_VELOCITY, config.MAX_VELOCITY)
        self.angular_velocity = np.clip(omega, -config.MAX_ANGULAR_VELOCITY, config.MAX_ANGULAR_VELOCITY)
    
    def update(self, dt: float, world_map=None) -> None:
        """Update robot pose using motion model"""
        old_pose = Pose(self.pose.x, self.pose.y, self.pose.theta)
        
        self._update_pose(self.true_pose, self.velocity, self.angular_velocity, dt)
        
        noisy_v, noisy_omega = add_motion_noise(
            self.velocity, self.angular_velocity,
            config.ODOM_NOISE_TRANSLATION, config.ODOM_NOISE_ROTATION
        )
        self._update_pose(self.pose, noisy_v, noisy_omega, dt)
        
        if world_map and self.check_collision(world_map):
            self.pose = old_pose
            self.true_pose = Pose(old_pose.x, old_pose.y, old_pose.theta)
            self.velocity = 0
            self.angular_velocity = 0
        
        self._record_path()
    
    def _update_pose(self, pose: Pose, v: float, omega: float, dt: float) -> None:
        """Update a pose using differential drive model"""
        if abs(omega) < 1e-6:
            pose.x += v * np.cos(pose.theta) * dt
            pose.y += v * np.sin(pose.theta) * dt
        else:
            pose.x += (v / omega) * (np.sin(pose.theta + omega * dt) - np.sin(pose.theta))
            pose.y += (v / omega) * (-np.cos(pose.theta + omega * dt) + np.cos(pose.theta))
        
        pose.theta += omega * dt
        pose.theta = normalize_angle(pose.theta)
    
    def _record_path(self) -> None:
        """Record robot path for visualization"""
        if len(self.path) == 0:
            self.path.append((self.pose.x, self.pose.y, self.pose.theta))
        else:
            last_pos = (self.path[-1][0], self.path[-1][1])
            current_pos = (self.pose.x, self.pose.y)
            if point_point_distance(last_pos, current_pos) > self.path_sample_distance:
                self.path.append((self.pose.x, self.pose.y, self.pose.theta))
    
    def check_collision(self, world_map) -> bool:
        """Check if robot collides with obstacles"""
        x, y = int(self.pose.x), int(self.pose.y)
        radius = int(self.radius * 10)
        
        for angle in np.linspace(0, 2*np.pi, 16):
            check_x = int(x + radius * np.cos(angle))
            check_y = int(y + radius * np.sin(angle))
            
            if (check_x < 0 or check_x >= world_map.get_width() or
                check_y < 0 or check_y >= world_map.get_height()):
                return True
            
            if world_map.get_at((check_x, check_y)) == (0, 0, 0):
                return True
        
        return False
    
    def get_sensor_data(self, world) -> Dict:
        """Get readings from all sensors"""
        sensor_data = {}
        for sensor in self.sensors:
            sensor_data[sensor.__class__.__name__] = sensor.sense()
        return sensor_data
    
    def get_pose(self) -> Pose:
        """Get current pose estimate"""
        return self.pose
    
    def get_true_pose(self) -> Pose:
        """Get ground truth pose"""
        return self.true_pose
    
    def reset(self) -> None:
        """Reset robot to initial state"""
        self.pose = Pose()
        self.true_pose = Pose()
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.path.clear()
    