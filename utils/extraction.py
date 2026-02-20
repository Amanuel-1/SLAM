from abc import ABC, abstractmethod
from typing import List, Tuple, Optional
from utils.geometry import (
    odr_fit, slope_intersept_to_general, general_to_slope_intercept,
    point_line_distance, point_point_distance, get_predicted_point,
    get_line_points, intersection_point_general, project_point_to_line
)


class LineExtraction(ABC):
    """
    Base class for all line extraction algorithms
    """
    @abstractmethod
    def extract(self, point_cloud: List[Tuple]):
        pass


class SplitMergeExtraction(LineExtraction):
    """
    Line segment extraction using seeded region growing algorithm.
    
    Implements Algorithms 1-3 from:
    "A line segment extraction algorithm using laser data based on seeded region growing"
    
    Algorithm 1: Seed-segment detection
    Algorithm 2: Region growing
    Algorithm 3: Overlap region processing
    """
    
    def __init__(self, 
                 epsilon: float = 0.05,      # ε: distance threshold for d2 (perpendicular distance to line)
                 delta: float = 0.05,         # δ: distance threshold for d1 (point to predicted point)
                 s_num: int = 3,              # S_num: number of points to fit seed segment
                 p_min: int = 3,              # P_min: minimum number of points for valid segment
                 l_min: float = 0.1,          # L_min: minimum length for valid segment
                 g_max: float = 0.5,          # G_max: maximum gap between consecutive points
                 angle_gap_max: float = 0.1,  # Maximum angular gap (radians) to prevent bridging
                 distance_jump_ratio: float = 1.5):  # Max ratio of distance change (prevents gaps)
        """
        Initialize split-and-merge extraction with algorithm parameters.
        
        Args:
            epsilon: Maximum perpendicular distance from point to line (ε in paper)
            delta: Maximum distance from point to predicted point (δ in paper)
            s_num: Number of points used to fit seed segment (S_num in paper)
            p_min: Minimum number of points required for valid segment (P_min in paper)
            l_min: Minimum segment length required (L_min in paper)
            g_max: Maximum gap between consecutive points (G_max in paper)
            angle_gap_max: Maximum angular gap (radians) to prevent bridging windows/doors
            distance_jump_ratio: Maximum ratio of distance change to detect gaps
        """
        self.EPSILON = epsilon
        self.DELTA = delta
        self.SNUM = s_num
        self.PMIN = p_min
        self.LMIN = l_min
        self.GMAX = g_max
        self.ANGLE_GAP_MAX = angle_gap_max
        self.DISTANCE_JUMP_RATIO = distance_jump_ratio
        
        # Internal state
        self.laser_points = []
        self.NP = 0
        self.seed_segments = []
        self.line_segments = []
        self.line_params = None
    
    def set_laser_points(self, laser_points: List[Tuple]):
        """
        Set the laser point cloud for extraction.
        
        Args:
            laser_points: List of tuples ((x, y), angle) representing laser scan points
        """
        self.laser_points = laser_points
        self.NP = len(laser_points)
        self.seed_segments = []
        self.line_segments = []
    
    def extract(self, point_cloud: List[Tuple]):
        """
        Main extraction method (to be implemented for full pipeline).
        """
        pass
    
    def detect_seed_segments(self, position: Tuple[float, float], break_point_index: int = 0) -> Optional[List]:
        """
        Algorithm 1: Seed-segment detection
        
        Finds a valid seed segment starting from break_point_index.
        A seed segment is a contiguous set of points that can be well-approximated
        by a straight line based on distance thresholds.
        
        Args:
            position: Sensor position (x, y) for computing predicted points
            break_point_index: Starting index to search for seed (default: 0)
        
        Returns:
            [points, predicted_points, (i, j)] if valid seed found, None otherwise
        """
        self.NP = max(0, self.NP)
        
        # Outer loop: iterate through possible starting indices
        # Paper: for i = 1 → (N_p - P_min) do
        for i in range(break_point_index, self.NP - self.PMIN + 1):
            flag = True
            pred_points = []
            j = i + self.SNUM
            
            # Check bounds
            if j > self.NP:
                continue
            
            has_gap = False
            for k in range(i, min(j - 1, len(self.laser_points) - 1)):
                point_k, angle_k = self.laser_points[k]
                point_k1, angle_k1 = self.laser_points[k + 1]
                
                if abs(angle_k1 - angle_k) > self.ANGLE_GAP_MAX:
                    has_gap = True
                    break
                
                dx = point_k1[0] - point_k[0]
                dy = point_k1[1] - point_k[1]
                spatial_gap_sq = dx * dx + dy * dy
                if spatial_gap_sq > self.GMAX * self.GMAX:
                    has_gap = True
                    break
                
                dx_k = point_k[0] - position[0]
                dy_k = point_k[1] - position[1]
                range_sq_k = dx_k * dx_k + dy_k * dy_k
                
                dx_k1 = point_k1[0] - position[0]
                dy_k1 = point_k1[1] - position[1]
                range_sq_k1 = dx_k1 * dx_k1 + dy_k1 * dy_k1
                
                if range_sq_k > 0 and range_sq_k1 > 0:
                    ratio_threshold_sq = self.DISTANCE_JUMP_RATIO * self.DISTANCE_JUMP_RATIO
                    if range_sq_k > range_sq_k1:
                        if range_sq_k > ratio_threshold_sq * range_sq_k1:
                            has_gap = True
                            break
                    else:
                        if range_sq_k1 > ratio_threshold_sq * range_sq_k:
                            has_gap = True
                            break
            
            if has_gap:
                continue
            
            # Fit line to seed segment points
            # Paper: fit Seed(i, j)
            m, b = odr_fit(self.laser_points[i:j])
            line_params = slope_intersept_to_general(m, b)
            
            # Inner loop: check each point in the seed segment
            # Paper: for k = i → j do
            for k in range(i, j):
                point, _ = self.laser_points[k]
                
                # Obtain predicted point P'_k
                # Paper: obtain the predicted point P'_k
                predicted_point = get_predicted_point(line_params, point, position)
                
                if predicted_point is None:
                    flag = False
                    break
                
                pred_points.append(predicted_point)
                
                # Calculate d1: distance from P_k to P'_k
                # Paper: d_1 ← distance from P_k to P'_k
                d1 = point_point_distance(predicted_point, point)
                
                # Check d1 threshold
                # Paper: if d_1 > δ then flag = false, break
                if d1 > self.DELTA:
                    flag = False
                    break
                
                # Calculate d2: distance from P_k to Seed(i, j)
                # Paper: d_2 ← distance from P_k to Seed(i, j)
                d2 = point_line_distance(point, line_params)
                
                # Check d2 threshold
                # Paper: if d_2 > ε then flag = false, break
                if d2 > self.EPSILON:
                    flag = False
                    break
            
            # If all points passed checks, return seed segment
            # Paper: if flag == true then return Seed(i, j)
            if flag:
                self.line_params = line_params
                return [self.laser_points[i:j], pred_points, (i, j)]
        
        return None
    
    def seed_segment_growing(self, indices: Tuple[int, int], break_point: int, position: Tuple[float, float] = (0, 0)) -> Optional[List]:
        """
        Algorithm 2: Region growing
        
        Expands a seed segment by incorporating adjacent laser points that satisfy
        distance constraints, growing both forward and backward from the seed.
        
        Args:
            indices: (i, j) - start and end indices of seed segment
            break_point: Index where previous segment ended (for backward growth limit)
        
        Returns:
            [points, two_points, endpoints, pf, line_params, (m, b)] if valid segment,
            None otherwise
        """
        i, j = indices
        
        # Initialize line from seed
        if self.line_params is None:
            m, b = odr_fit(self.laser_points[i:j])
            line = slope_intersept_to_general(m, b)
        else:
            line = self.line_params
        
        # Initialize forward and backward pointers
        # Paper: P_f ← j+1, P_b ← i-1
        pf = j + 1
        pb = i - 1
        
        # Forward growth
        last_included_point = self.laser_points[j - 1] if j > 0 and j - 1 < len(self.laser_points) else None
        
        sensor_x, sensor_y = position if position else (0, 0)
        gmax_sq = self.GMAX * self.GMAX
        ratio_threshold_sq = self.DISTANCE_JUMP_RATIO * self.DISTANCE_JUMP_RATIO
        
        while pf < self.NP:
            if pf >= len(self.laser_points):
                break
            
            point, angle = self.laser_points[pf]
            
            if last_included_point is not None:
                prev_point, prev_angle = last_included_point
                
                if abs(angle - prev_angle) > self.ANGLE_GAP_MAX:
                    break
                
                dx = point[0] - prev_point[0]
                dy = point[1] - prev_point[1]
                if dx * dx + dy * dy > gmax_sq:
                    break
                
                dx_prev = prev_point[0] - sensor_x
                dy_prev = prev_point[1] - sensor_y
                range_sq_prev = dx_prev * dx_prev + dy_prev * dy_prev
                
                dx_curr = point[0] - sensor_x
                dy_curr = point[1] - sensor_y
                range_sq_curr = dx_curr * dx_curr + dy_curr * dy_curr
                
                if range_sq_prev > 0 and range_sq_curr > 0:
                    if range_sq_prev > range_sq_curr:
                        if range_sq_prev > ratio_threshold_sq * range_sq_curr:
                            break
                    else:
                        if range_sq_curr > ratio_threshold_sq * range_sq_prev:
                            break
            
            d = point_line_distance(point, line)
            if d > self.EPSILON:
                break
            
            last_included_point = self.laser_points[pf]
            pf += 1
            
            if pf <= len(self.laser_points):
                try:
                    m, b = odr_fit(self.laser_points[pb + 1:pf])
                    line = slope_intersept_to_general(m, b)
                except (IndexError, ValueError):
                    break
        
        # Adjust pf to last successfully included point
        # Paper: P_f ← P_f - 1
        # Ensure pf is within valid array bounds
        pf = min(pf - 1, len(self.laser_points) - 1)
        pf = max(pf, j)  # Ensure pf is at least at the seed end index
        # Final bounds check
        if pf >= len(self.laser_points):
            pf = len(self.laser_points) - 1
        
        # Backward growth
        last_included_point_backward = self.laser_points[i] if i < len(self.laser_points) else None
        
        while pb >= break_point and pb >= 0:
            if pb >= len(self.laser_points):
                break
            
            point, angle = self.laser_points[pb]
            
            if last_included_point_backward is not None:
                next_point, next_angle = last_included_point_backward
                
                if abs(angle - next_angle) > self.ANGLE_GAP_MAX:
                    break
                
                dx = point[0] - next_point[0]
                dy = point[1] - next_point[1]
                if dx * dx + dy * dy > gmax_sq:
                    break
                
                dx_point = point[0] - sensor_x
                dy_point = point[1] - sensor_y
                range_sq_point = dx_point * dx_point + dy_point * dy_point
                
                dx_next = next_point[0] - sensor_x
                dy_next = next_point[1] - sensor_y
                range_sq_next = dx_next * dx_next + dy_next * dy_next
                
                if range_sq_point > 0 and range_sq_next > 0:
                    if range_sq_point > range_sq_next:
                        if range_sq_point > ratio_threshold_sq * range_sq_next:
                            break
                    else:
                        if range_sq_next > ratio_threshold_sq * range_sq_point:
                            break
            
            d = point_line_distance(point, line)
            if d > self.EPSILON:
                break
            
            last_included_point_backward = self.laser_points[pb]
            pb -= 1
            
            if pb >= break_point and pb >= 0 and pf + 1 <= len(self.laser_points):
                try:
                    m, b = odr_fit(self.laser_points[pb + 1:pf + 1])
                    line = slope_intersept_to_general(m, b)
                except (IndexError, ValueError):
                    break
        
        # Adjust pb to first successfully included point
        # Paper: P_b ← P_b + 1
        pb = max(pb + 1, 0)
        # pb can be less than i if backward growth succeeded
        
        # Calculate final segment properties
        # Paper: obtain L_l, P_l from Line(P_b, P_f)
        if pb >= pf or pb >= len(self.laser_points) or pf >= len(self.laser_points):
            return None
        
        # Ensure indices are valid before accessing
        if pb < 0 or pf < 0 or pb >= len(self.laser_points) or pf >= len(self.laser_points):
            return None
        
        start_point = self.laser_points[pb][0]
        end_point = self.laser_points[pf][0]
        LR = point_point_distance(start_point, end_point)
        PR = len(self.laser_points[pb:pf + 1])
        
        # Check minimum length and point count constraints
        # Paper: if (L_l >= L_min) and (P_l >= P_min) then return Line(P_b, P_f)
        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.line_params = line
            m, b = general_to_slope_intercept(*line)
            two_points = get_line_points(m, b)
            
            # Store segment information
            segment_info = {
                'indices': (pb, pf),
                'points': self.laser_points[pb:pf + 1],
                'line_params': line,
                'slope_intercept': (m, b),
                'endpoints': (start_point, end_point),
                'length': LR,
                'point_count': PR
            }
            
            self.line_segments.append(segment_info)
            
            return [
                self.laser_points[pb:pf + 1],
                two_points,
                (start_point, end_point),
                pf,
                line,
                (m, b)
            ]
        else:
            return None
    
    def process_overlap_regions(self) -> List[dict]:
        """
        Algorithm 3: Overlap region processing
        
        Processes overlapping or collinear line segments to resolve conflicts
        and adjust segment boundaries based on point-to-line distances.
        
        Returns:
            List of processed line segments without overlaps
        """
        if len(self.line_segments) <= 1:
            return self.line_segments
        
        # Sort segments by start index
        sorted_segments = sorted(self.line_segments, key=lambda s: s['indices'][0])
        
        # Process adjacent pairs
        # Paper: for i = 1 → N_l - 1, j = i + 1 do
        i = 0
        while i < len(sorted_segments) - 1:
            seg_i = sorted_segments[i]
            seg_j = sorted_segments[i + 1]
            
            m1, n1 = seg_i['indices']  # Endpoint indices for Line_i
            m2, n2 = seg_j['indices']  # Endpoint indices for Line_j
            
            # Check for overlap: m2 <= n1 means segments overlap
            # Paper: if m2 <= n1 then
            if m2 <= n1:
                line_i_params = seg_i['line_params']
                line_j_params = seg_j['line_params']
                
                # Process overlapping region
                # Paper: for k = m2 → n1 do
                k = m2
                boundary_found = False
                for k in range(m2, min(n1, n2) + 1):
                    if k >= len(self.laser_points):
                        break
                    
                    point = self.laser_points[k][0]
                    
                    # Calculate distances to both lines
                    # Paper: d_i^k ← distance from P_k to Line_i
                    # Paper: d_j^k ← distance from P_k to Line_j
                    d_i_k = point_line_distance(point, line_i_params)
                    d_j_k = point_line_distance(point, line_j_params)
                    
                    # If point is closer to Line_j, break (found boundary)
                    # Paper: if d_i^k >= d_j^k then break
                    if d_i_k >= d_j_k:
                        boundary_found = True
                        break
                
                # Adjust segment boundaries
                # Paper: n1 ← k - 1, m2 ← k
                if boundary_found:
                    n1_new = k - 1
                    m2_new = k
                else:
                    # No clear boundary found, use original boundaries
                    n1_new = n1
                    m2_new = m2
                
                # Ensure valid indices
                n1_new = max(m1, min(n1_new, len(self.laser_points) - 1))
                m2_new = max(m2_new, 0)
                
                # Refit both segments with adjusted boundaries
                # Paper: refit Line(m1, n1), refit Line(m2, n2)
                if m1 <= n1_new and n1_new < len(self.laser_points) and (n1_new - m1 + 1) >= self.PMIN:
                    try:
                        m, b = odr_fit(self.laser_points[m1:n1_new + 1])
                        sorted_segments[i]['line_params'] = slope_intersept_to_general(m, b)
                        sorted_segments[i]['indices'] = (m1, n1_new)
                        sorted_segments[i]['slope_intercept'] = (m, b)
                        # Update endpoints
                        sorted_segments[i]['endpoints'] = (
                            self.laser_points[m1][0],
                            self.laser_points[n1_new][0]
                        )
                    except Exception:
                        pass  # Keep original segment if refitting fails
                
                if m2_new <= n2 and m2_new < len(self.laser_points) and (n2 - m2_new + 1) >= self.PMIN:
                    try:
                        m, b = odr_fit(self.laser_points[m2_new:n2 + 1])
                        sorted_segments[i + 1]['line_params'] = slope_intersept_to_general(m, b)
                        sorted_segments[i + 1]['indices'] = (m2_new, n2)
                        sorted_segments[i + 1]['slope_intercept'] = (m, b)
                        # Update endpoints
                        sorted_segments[i + 1]['endpoints'] = (
                            self.laser_points[m2_new][0],
                            self.laser_points[n2][0]
                        )
                    except Exception:
                        pass  # Keep original segment if refitting fails
            else:
                # No overlap, move to next pair
                pass
            
            i += 1
        
        return sorted_segments
    
    def validate_segment_continuity(self, segment: dict, sensor_position: Tuple[float, float] = None) -> bool:
        """
        Lightweight validation that a line segment represents a continuous surface.
        Samples key points to check for outliers that indicate bridging across gaps.
        """
        line_params = segment.get('line_params')
        points = segment.get('points', [])
        
        if line_params is None or len(points) < 3:
            return True
        
        sample_indices = [0, len(points) // 2, len(points) - 1]
        if len(points) > 10:
            sample_indices.extend([len(points) // 4, 3 * len(points) // 4])
        
        max_dist = 0.0
        for idx in sample_indices:
            if idx < len(points):
                point = points[idx][0]
                dist = point_line_distance(point, line_params)
                if dist > max_dist:
                    max_dist = dist
        
        if max_dist > 2.0 * self.EPSILON:
            return False
        
        return True


class IncrementalExtraction(LineExtraction):
    """Incremental line extraction algorithm (stub)"""
    
    def _check_line_condition(self, line_model):
        pass
    
    def extract(self, point_cloud: List[Tuple]):
        pass
