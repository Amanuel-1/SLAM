"""World simulation environment manager"""
from typing import Optional, List, Dict
from utils.geometry import polar2p
import pygame



class World:
    """Manages the simulation environment"""
    
    def __init__(self, map_path:str , width: int = 1200, height: int = 700):
        self.map_path = map_path
        
        # Load image first to get its dimensions
        temp_img = pygame.image.load(self.map_path)
        self.width = temp_img.get_width()
        self.height = temp_img.get_height()
        
        self.robot = None
        self.landmarks = []
        self.map = pygame.display.set_mode(size=(self.width, self.height))
        pygame.display.set_caption("SLAM Simulation")
        self.sensor_map = []
        self.point_cloud = []

        self._init_world()

        self.time = 0.0
        print(f"""
            this is a test and the dimention of the image is {self.width}x{self.height}
        """)

    
    def _init_world(self):
        try:
            map_blob = pygame.image.load(self.map_path)
            self.width = map_blob.get_width()
            self.height = map_blob.get_height()
            

            # apply the map into the environment
            self.map.blit(map_blob, (0, 0))
            # pygame.display.flip()  
            print(f"Map loaded successfully from {self.map_path}")
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
            self.map.set_at((int(x), int(y)), (255, 0, 0))







         

    
    def reset(self) -> None:
        """Reset world state"""
        self.time = 0.0
        if self.robot:
            self.robot.reset()
