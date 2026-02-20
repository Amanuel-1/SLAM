"""Noise generation utilities for sensor simulation"""
import numpy as np
import numpy.typing as np_type
import config

def add_gaussian_noise(distance: float, angle: float, 
                       sigma: np_type.NDArray = config.LIDAR_NOISE_SIGMA,
                       enable_noise: bool = True) -> tuple:
    """
    Add Gaussian noise to lidar sensor data.
    
    Args:
        distance: True distance measurement
        angle: True angle measurement
        sigma: Standard deviations [distance_sigma, angle_sigma]
        enable_noise: If False, return original values without noise
    
    Returns:
        (noisy_distance, noisy_angle) tuple
    """
    if not enable_noise:
        return (distance, angle)
    
    # Use independent Gaussian noise for distance and angle
    # This is more appropriate than multivariate_normal for independent variables
    noisy_distance = distance + np.random.normal(0, sigma[0])
    noisy_angle = angle + np.random.normal(0, sigma[1])
    
    # Ensure distance is positive
    noisy_distance = max(0.0, noisy_distance)
    
    return (noisy_distance, noisy_angle)


def add_motion_noise(velocity: float, angular_velocity: float, 
                     trans_std: float, rot_std: float) -> tuple:
    """Add noise to motion commands"""
    noisy_v = add_gaussian_noise(velocity, trans_std)
    noisy_omega = add_gaussian_noise(angular_velocity, rot_std)
    return noisy_v, noisy_omega
