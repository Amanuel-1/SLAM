"""Data association algorithms"""
import numpy as np
from typing import Optional, List, Tuple


def nearest_neighbor_association(observation: Tuple[float, float],
                                 landmarks: List[Tuple[float, float, int]],
                                 threshold: float) -> Optional[int]:
    """
    Find nearest landmark to observation using Euclidean distance.
    
    Args:
        observation: (x, y) observed point
        landmarks: List of (x, y, landmark_id) tuples
        threshold: Maximum distance for association
    
    Returns:
        landmark_id if match found, None otherwise
    """
    if not landmarks:
        return None
    
    obs_x, obs_y = observation
    min_dist = float('inf')
    best_match = None
    
    for lm_x, lm_y, lm_id in landmarks:
        dist = np.sqrt((obs_x - lm_x)**2 + (obs_y - lm_y)**2)
        if dist < min_dist and dist < threshold:
            min_dist = dist
            best_match = lm_id
    
    return best_match


def mahalanobis_distance(obs: np.ndarray, landmark: np.ndarray, 
                        covariance: np.ndarray) -> float:
    """
    Compute Mahalanobis distance for data association.
    
    Args:
        obs: Observation vector (numpy array)
        landmark: Landmark vector (numpy array)
        covariance: Covariance matrix (numpy array)
    
    Returns:
        Mahalanobis distance
    """
    diff = obs - landmark
    try:
        inv_cov = np.linalg.inv(covariance)
        mahal_dist = np.sqrt(diff.T @ inv_cov @ diff)
        return float(mahal_dist)
    except np.linalg.LinAlgError:
        # If covariance is singular, fall back to Euclidean distance
        return float(np.linalg.norm(diff))
