"""Noise generation utilities for sensor simulation"""
import numpy as np
import numpy.typing as np_type
import config

def add_gaussian_noise(distance: float,angle:float,sigma:np_type.NDArray= config.LIDAR_NOISE_SIGMA) -> float:
    """Add Gaussian noise to a sensor data"""
    mu = np.array([distance,angle])
    covariance = np.diag(sigma**2)
    
    distance,angle = np.random.multivariate_normal(mean=mu,cov=covariance)

    return (distance,angle)


def add_motion_noise(velocity: float, angular_velocity: float, 
                     trans_std: float, rot_std: float) -> tuple:
    """Add noise to motion commands"""
    noisy_v = add_gaussian_noise(velocity, trans_std)
    noisy_omega = add_gaussian_noise(angular_velocity, rot_std)
    return noisy_v, noisy_omega
