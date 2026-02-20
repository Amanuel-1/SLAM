"""World simulation environment manager"""
from typing import Optional, List, Dict
from utils.geometry import polar2p
import pygame
import config



class World:
    """Manages the simulation environment"""
    
    def __init__(self, map_path:str , width: int = None, height: int = None):
        self.map_path = map_path
        
        # Use config dimensions if not provided
        if width is None:
            width = config.WORLD_WIDTH
        if height is None:
            height = config.WORLD_HEIGHT
            
        self.width = width
        self.height = height
        
        self.robot = None
        self.landmarks = []
        self.map = pygame.display.set_mode(size=(self.width, self.height))
        pygame.display.set_caption("SLAM Simulation")
        self.sensor_map = []
        self.point_cloud = []

        self._init_world()

        self.time = 0.0
        print(f"""
            Window dimensions: {self.width}x{self.height}
        """)

    
    def _init_world(self):
        try:
            map_blob = pygame.image.load(self.map_path)
            original_width = map_blob.get_width()
            original_height = map_blob.get_height()
            
            # Scale the map to fit the window dimensions
            map_blob = pygame.transform.scale(map_blob, (self.width, self.height))

            # apply the map into the environment
            self.map.blit(map_blob, (0, 0))
            print(f"Map loaded and scaled from {original_width}x{original_height} to {self.width}x{self.height}")
        except Exception as e:
            print(f"Error loading map: {e}")
            # Fill with white background as fallback
            self.map.fill((255, 255, 255))
            pygame.display.flip()


    def create_point_cloud(self, data):
        """Create point cloud from sensor data (deprecated - use transform_polar_point_set instead)"""
        for (distance, angle, sensor_position) in data:
            data_point = polar2p(distance, angle, sensor_position)
            self.point_cloud.append(data_point)
    
    def clear_point_cloud(self):
        """Clear the point cloud"""
        self.point_cloud = []

    def visualize_point_cloud(self):
        for (x, y) in self.point_cloud:
            self.map.set_at((int(x), int(y)), (50, 200, 50))
    
    def visualize_line_segments(self, line_segments, color=(255, 0, 0), thickness=2):
        """
        Visualize extracted line segments on the map.
        
        Args:
            line_segments: List of segment dictionaries with 'endpoints' key
            color: RGB color tuple for line segments
            thickness: Line thickness in pixels
        """
        if not line_segments:
            return
        
        for segment in line_segments:
            if segment and 'endpoints' in segment:
                p1, p2 = segment['endpoints']
                if p1 and p2:
                    try:
                        pygame.draw.line(self.map, color, 
                                       (int(p1[0]), int(p1[1])), 
                                       (int(p2[0]), int(p2[1])), 
                                       thickness)
                    except (ValueError, TypeError):
                        pass  # Skip invalid coordinates
    
    def visualize_landmarks(self, landmarks, color=(0, 255, 255), thickness=3):
        """
        Visualize line segment landmarks on the map.
        
        Args:
            landmarks: List of LineLandmark objects
            color: RGB color tuple for landmarks
            thickness: Line thickness in pixels
        """
        if not landmarks:
            return
        
        for landmark in landmarks:
            if landmark and landmark.endpoints:
                p1, p2 = landmark.endpoints
                if p1 and p2:
                    try:
                        # Draw landmark line
                        pygame.draw.line(self.map, color,
                                       (int(p1[0]), int(p1[1])),
                                       (int(p2[0]), int(p2[1])),
                                       thickness)
                        
                        # Draw midpoint marker
                        mid_x, mid_y = int(landmark.midpoint[0]), int(landmark.midpoint[1])
                        pygame.draw.circle(self.map, color, (mid_x, mid_y), 5)
                    except (ValueError, TypeError):
                        pass  # Skip invalid coordinates
    
    def visualize_seed_segments(self, seed_data, color=(255, 255, 0), thickness=1):
        """
        Visualize seed segments for debugging.
        
        Args:
            seed_data: List of seed segment data [points, predicted_points, (i, j)]
            color: RGB color tuple for seed segments
            thickness: Line thickness in pixels
        """
        if not seed_data:
            return
        
        for seed in seed_data:
            if seed and len(seed) >= 3:
                points, pred_points, (i, j) = seed
                if points and len(points) >= 2:
                    for idx in range(len(points) - 1):
                        if idx < len(points) and idx + 1 < len(points):
                            p1 = points[idx][0]
                            p2 = points[idx + 1][0]
                            if p1 and p2:
                                try:
                                    pygame.draw.line(self.map, color,
                                                   (int(p1[0]), int(p1[1])),
                                                   (int(p2[0]), int(p2[1])),
                                                   thickness)
                                except (ValueError, TypeError):
                                    pass
    
    def visualize_robot(self, robot, color=(0, 0, 255)):
        """Draw robot as circle with heading indicator"""
        import math
        x, y = int(robot.pose.x), int(robot.pose.y)
        radius = 15
        
        pygame.draw.circle(self.map, color, (x, y), radius, 0)
        
        pygame.draw.circle(self.map, (255, 255, 255), (x, y), radius, 2)
        
        end_x = x + radius * 2 * math.cos(robot.pose.theta)
        end_y = y + radius * 2 * math.sin(robot.pose.theta)
        pygame.draw.line(self.map, (255, 255, 0), (x, y), 
                        (int(end_x), int(end_y)), 3)
    
    def visualize_robot_path(self, path, color=(255, 0, 255)):
        """Draw robot's traveled path"""
        if len(path) < 2:
            return
        
        for i in range(len(path) - 1):
            p1 = (int(path[i][0]), int(path[i][1]))
            p2 = (int(path[i+1][0]), int(path[i+1][1]))
            pygame.draw.line(self.map, color, p1, p2, 2)
        
        # for (x, y, _) in path:
        #     pygame.draw.circle(self.map, color, (int(x), int(y)), 2)







         

    
    def reset(self) -> None:
        """Reset world state"""
        self.time = 0.0
        if self.robot:
            self.robot.reset()
