"""Robot model with pose and motion"""
from dataclasses import dataclass
from typing import List, Dict, Tuple
import numpy as np
from utils.geometry import normalize_angle
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
    
    def add_sensor(self, sensor) -> None:
        """Add a sensor to the robot"""
        self.sensors.append(sensor)
    
    def set_velocity(self, v: float, omega: float) -> None:
        """Set robot velocity commands"""
        self.velocity = np.clip(v, -config.MAX_VELOCITY, config.MAX_VELOCITY)
        self.angular_velocity = np.clip(omega, -config.MAX_ANGULAR_VELOCITY, config.MAX_ANGULAR_VELOCITY)
    
    def update(self, dt: float, world) -> None:
        """Update robot pose using motion model"""
        # True motion (no noise)
        self._update_pose(self.true_pose, self.velocity, self.angular_velocity, dt)
        
        # Noisy motion for odometry
        noisy_v, noisy_omega = add_motion_noise(
            self.velocity, self.angular_velocity,
            config.ODOM_NOISE_TRANSLATION, config.ODOM_NOISE_ROTATION
        )
        self._update_pose(self.pose, noisy_v, noisy_omega, dt)
    
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
    
    def get_sensor_data(self, world) -> Dict:
        """Get readings from all sensors"""
        sensor_data = {}
        for sensor in self.sensors:
            sensor_data[sensor.__class__.__name__] = sensor.sense(self, world)
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
