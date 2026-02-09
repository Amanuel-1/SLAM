"""Data association algorithms"""
import numpy as np
from typing import Optional, List, Tuple


def nearest_neighbor_association(observation: Tuple[float, float],
                                 landmarks: List[Tuple[float, float, int]],
                                 threshold: float) -> Optional[int]:
    """Find nearest landmark to observation"""
    pass


def mahalanobis_distance(obs: np.ndarray, landmark: np.ndarray, 
                        covariance: np.ndarray) -> float:
    """Compute Mahalanobis distance for data association"""
    pass
