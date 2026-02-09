"""Geometric computations for SLAM"""
import numpy as np


def compute_jacobian(state: np.ndarray, control: tuple) -> np.ndarray:
    """Compute Jacobian for motion model"""
    pass


def compute_observation_jacobian(state: np.ndarray, landmark_idx: int) -> np.ndarray:
    """Compute Jacobian for observation model"""
    pass
