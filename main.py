"""Main entry point for SLAM simulation"""
import pygame
import numpy as np
from world import World
from robot.sensors import Lidar
import config
import sys
import traceback



def main():
    """Main simulation loop"""
    try:
        # Initialize pygame
        pygame.init()
        print("Pygame initialized", flush=True)
        
        # Create world
        print("Creating world...", flush=True)
        world = World(map_path="world/maps/indoor1.png")
        
        # store original map for sensor scanning
        original_map = world.map.copy()
        scanner = Lidar(original_map)
        
        # Fill display with black
        world.map.fill((0,0,0))

        

        
        
        # Simulation loop
        running = True
        
        dt = 1.0 / config.FPS
        print("Starting main loop...", flush=True)

        
        while running:
            sensor_on = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if pygame.mouse.get_focused():
                    print("SENSOR-ON")
                    sensor_on = True 
                elif not pygame.mouse.get_focused():
                    print("SENSOR-OFF")
                    sensor_on = False

            if sensor_on:
                position = pygame.mouse.get_pos()
                scanner.pos = tuple(position)
                sensor_output_data = scanner.sense()
                print(f"Sensor data points: {len(sensor_output_data)}", flush=True)
                if sensor_output_data:
                    world.create_point_cloud(sensor_output_data)
                    world.visualize_point_cloud()
            
            pygame.display.update()
        
        print("Exiting...", flush=True)
        pygame.quit()
        
    except Exception as e:
        print(f"ERROR: {e}", flush=True)
        traceback.print_exc()
        pygame.quit()
        sys.exit(1)


if __name__ == "__main__":
    main()
