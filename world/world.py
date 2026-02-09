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

        for (distance,angle,sensor_position) in data:
            data_point = polar2p(distance,angle,sensor_position)

            self.point_cloud.append(data_point)

    def visualize_point_cloud(self):
        for (x,y) in self.point_cloud:
            self.map.set_at((int(x), int(y)), (50, 200, 50))







         

    
    def reset(self) -> None:
        """Reset world state"""
        self.time = 0.0
        if self.robot:
            self.robot.reset()
