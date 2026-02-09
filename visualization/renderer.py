"""Pygame-based visualization"""
import pygame
import numpy as np
from typing import Optional
import config


class Renderer:
    """Handles visualization of world, robot, and SLAM estimates"""
    
    def __init__(self, width: int, height: int):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SLAM Simulation")
        self.clock = pygame.time.Clock()
        self.world = None
        self.slam = None
    
    def set_world(self, world) -> None:
        """Set the world to render"""
        self.world = world
    
    def set_slam(self, slam) -> None:
        """Set the SLAM algorithm to visualize"""
        self.slam = slam
    
    def draw(self) -> None:
        """Draw everything"""
        self.screen.fill(config.BACKGROUND_COLOR)
        
        if self.world:
            self.draw_map()
            self.draw_landmarks(self.world.landmarks, config.LANDMARK_COLOR)
            
            if self.world.robot:
                self.draw_robot(self.world.robot.get_true_pose(), config.ROBOT_COLOR)
                self.draw_sensor_rays()
        
        if self.slam:
            estimated_pose = self.slam.get_estimated_pose()
            self.draw_robot_estimate(estimated_pose, config.ESTIMATED_ROBOT_COLOR)
            
            estimated_landmarks = self.slam.get_estimated_landmarks()
            self.draw_estimated_landmarks(estimated_landmarks)
        
        pygame.display.flip()
        self.clock.tick(config.FPS)
    
    def draw_map(self) -> None:
        """Draw the occupancy grid"""
        pass
    
    def draw_robot(self, pose, color) -> None:
        """Draw robot at given pose"""
        x, y, theta = pose.x, pose.y, pose.theta
        screen_x, screen_y = self.world_to_screen(x, y)
        
        pygame.draw.circle(self.screen, color, (int(screen_x), int(screen_y)), 10)
        
        # Draw heading
        end_x = screen_x + 15 * np.cos(theta)
        end_y = screen_y + 15 * np.sin(theta)
        pygame.draw.line(self.screen, color, (screen_x, screen_y), (end_x, end_y), 2)
    
    def draw_robot_estimate(self, pose_tuple, color) -> None:
        """Draw estimated robot pose"""
        pass
    
    def draw_landmarks(self, landmarks, color) -> None:
        """Draw true landmarks"""
        for landmark in landmarks:
            screen_x, screen_y = self.world_to_screen(landmark.x, landmark.y)
            pygame.draw.circle(self.screen, color, (int(screen_x), int(screen_y)), 5)
    
    def draw_estimated_landmarks(self, landmarks) -> None:
        """Draw estimated landmarks"""
        pass
    
    def draw_sensor_rays(self) -> None:
        """Draw lidar rays"""
        pass
    
    def world_to_screen(self, x: float, y: float) -> tuple:
        """Convert world coordinates to screen coordinates"""
        screen_x = x * 50 + self.width // 2
        screen_y = self.height // 2 - y * 50
        return screen_x, screen_y
    
    def handle_events(self) -> bool:
        """Handle pygame events, return False to quit"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True
    
    def close(self) -> None:
        """Clean up pygame"""
        pygame.quit()
