"""Landmark management for line segment features"""
import numpy as np
from typing import List, Dict, Tuple, Optional
from utils.geometry import point_line_distance, point_point_distance
import config


class LineLandmark:
    """Represents a line segment landmark"""
    
    def __init__(self, landmark_id: int, endpoints: Tuple[Tuple[float, float], Tuple[float, float]],
                 line_params: Tuple[float, float, float], length: float, point_count: int):
        """
        Initialize a line landmark.
        
        Args:
            landmark_id: Unique identifier for this landmark
            endpoints: ((x1, y1), (x2, y2)) - endpoints of the line segment
            line_params: (A, B, C) - line equation in general form Ax + By + C = 0
            length: Length of the line segment
            point_count: Number of points used to form this segment
        """
        self.id = landmark_id
        self.endpoints = endpoints
        self.line_params = line_params
        self.length = length
        self.point_count = point_count
        self.observation_count = 1  # Number of times this landmark has been observed
        self.last_seen = 0  # Frame/timestamp when last observed
        
        # Compute midpoint for distance calculations
        p1, p2 = endpoints
        self.midpoint = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    
    def is_strong_feature(self, min_length: float = None, min_observations: int = 2) -> bool:
        """
        Check if this landmark represents a strong feature.
        
        Args:
            min_length: Minimum length threshold (uses config if None)
            min_observations: Minimum number of observations
        
        Returns:
            True if this is a strong feature
        """
        if min_length is None:
            min_length = config.LINE_EXTRACTION_L_MIN
        
        return (self.length >= min_length and 
                self.observation_count >= min_observations and
                self.point_count >= config.LINE_EXTRACTION_P_MIN)
    
    def distance_to_point(self, point: Tuple[float, float]) -> float:
        """Calculate perpendicular distance from point to this line"""
        return point_line_distance(point, self.line_params)
    
    def distance_to_midpoint(self, point: Tuple[float, float]) -> float:
        """Calculate distance from point to line segment midpoint"""
        return point_point_distance(point, self.midpoint)
    
    def similarity_score(self, observed_segment: Dict) -> float:
        """
        Calculate similarity score between this landmark and an observed segment.
        Lower score = more similar.
        
        Args:
            observed_segment: Dictionary with 'endpoints', 'line_params', 'length'
        
        Returns:
            Similarity score (lower is better)
        """
        obs_endpoints = observed_segment['endpoints']
        obs_line_params = observed_segment['line_params']
        obs_length = observed_segment.get('length', 0)
        
        # Distance between midpoints
        obs_midpoint = ((obs_endpoints[0][0] + obs_endpoints[1][0]) / 2,
                       (obs_endpoints[0][1] + obs_endpoints[1][1]) / 2)
        midpoint_dist = point_point_distance(self.midpoint, obs_midpoint)
        
        # Distance between line parameters (normalized)
        # Compare perpendicular distance at midpoint
        line_dist = abs(point_line_distance(obs_midpoint, self.line_params))
        
        # Length difference
        length_diff = abs(self.length - obs_length) / max(self.length, obs_length, 1.0)
        
        # Combined score (weighted)
        score = (midpoint_dist * 0.4 + 
                line_dist * 0.4 + 
                length_diff * 20.0 * 0.2)
        
        return score


class LineLandmarkManager:
    """Manages line segment landmarks with data association"""
    
    def __init__(self, 
                 association_threshold: float = None,
                 min_landmark_length: float = None,
                 min_observations: int = 2):
        """
        Initialize landmark manager.
        
        Args:
            association_threshold: Maximum similarity score for association (uses config if None)
            min_landmark_length: Minimum length for new landmarks (uses config if None)
            min_observations: Minimum observations before landmark is considered strong
        """
        self.landmarks: Dict[int, LineLandmark] = {}
        self.next_landmark_id = 0
        
        if association_threshold is None:
            association_threshold = config.DATA_ASSOCIATION_THRESHOLD
        if min_landmark_length is None:
            min_landmark_length = config.LINE_EXTRACTION_L_MIN
        
        self.association_threshold = association_threshold
        self.min_landmark_length = min_landmark_length
        self.min_observations = min_observations
    
    def associate_observations(self, observed_segments: List[Dict], 
                               current_frame: int = 0) -> Dict[int, Optional[int]]:
        """
        Associate observed line segments with existing landmarks.
        
        Args:
            observed_segments: List of segment dictionaries from line extraction
            current_frame: Current frame/timestamp
        
        Returns:
            Dictionary mapping observation index to landmark_id (None if new landmark)
        """
        associations = {}
        
        for obs_idx, obs_segment in enumerate(observed_segments):
            best_match_id = None
            best_score = float('inf')
            
            # Find best matching landmark
            for landmark_id, landmark in self.landmarks.items():
                score = landmark.similarity_score(obs_segment)
                if score < best_score and score < self.association_threshold:
                    best_score = score
                    best_match_id = landmark_id
            
            if best_match_id is not None:
                # Update existing landmark
                landmark = self.landmarks[best_match_id]
                landmark.observation_count += 1
                landmark.last_seen = current_frame
                # Optionally update endpoints/line_params with weighted average
                associations[obs_idx] = best_match_id
            else:
                # Create new landmark if segment is long enough
                if obs_segment.get('length', 0) >= self.min_landmark_length:
                    landmark_id = self._create_landmark(obs_segment, current_frame)
                    associations[obs_idx] = landmark_id
                else:
                    associations[obs_idx] = None
        
        return associations
    
    def _create_landmark(self, segment: Dict, frame: int) -> int:
        """Create a new landmark from a segment"""
        landmark_id = self.next_landmark_id
        self.next_landmark_id += 1
        
        landmark = LineLandmark(
            landmark_id=landmark_id,
            endpoints=segment['endpoints'],
            line_params=segment['line_params'],
            length=segment.get('length', 0),
            point_count=segment.get('point_count', 0)
        )
        landmark.last_seen = frame
        
        self.landmarks[landmark_id] = landmark
        return landmark_id
    
    def get_strong_landmarks(self) -> List[LineLandmark]:
        """Get all landmarks that are considered strong features"""
        return [lm for lm in self.landmarks.values() 
                if lm.is_strong_feature(self.min_landmark_length, self.min_observations)]
    
    def get_all_landmarks(self) -> List[LineLandmark]:
        """Get all landmarks"""
        return list(self.landmarks.values())
    
    def remove_old_landmarks(self, max_age: int = 10, current_frame: int = 0):
        """Remove landmarks that haven't been seen recently"""
        to_remove = [lid for lid, lm in self.landmarks.items() 
                    if current_frame - lm.last_seen > max_age and not lm.is_strong_feature()]
        for lid in to_remove:
            del self.landmarks[lid]
    
    def clear(self):
        """Clear all landmarks"""
        self.landmarks.clear()
        self.next_landmark_id = 0
