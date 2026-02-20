"""Main entry point for SLAM simulation"""
import pygame
import numpy as np
from world import World
from robot.sensors import Lidar
from utils.extraction import SplitMergeExtraction
from utils.geometry import transform_polar_point_set
from slam.line_landmark_manager import LineLandmarkManager
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
        
        # Initialize line extraction algorithm
        line_extractor = SplitMergeExtraction(
            epsilon=config.LINE_EXTRACTION_EPSILON,
            delta=config.LINE_EXTRACTION_DELTA,
            s_num=config.LINE_EXTRACTION_S_NUM,
            p_min=config.LINE_EXTRACTION_P_MIN,
            l_min=config.LINE_EXTRACTION_L_MIN,
            g_max=config.LINE_EXTRACTION_G_MAX,
            angle_gap_max=config.LINE_EXTRACTION_ANGLE_GAP_MAX,
            distance_jump_ratio=config.LINE_EXTRACTION_DISTANCE_JUMP_RATIO
        )
        
        # Initialize landmark manager for data association
        landmark_manager = LineLandmarkManager(
            association_threshold=config.DATA_ASSOCIATION_THRESHOLD,
            min_landmark_length=config.LINE_EXTRACTION_L_MIN,
            min_observations=2  # Require 2 observations before considering landmark "strong"
        )
        
        # Frame counter for landmark tracking
        frame_count = 0
        
        # Visualization mode: 'points', 'seeds', 'lines', or 'all'
        # Press '1' for points only, '2' for seeds, '3' for lines, '4' for all
        viz_mode = 'all'
        
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
                elif event.type == pygame.KEYDOWN:
                    # Toggle visualization modes
                    if event.key == pygame.K_1:
                        viz_mode = 'points'
                        print("Visualization mode: Points only", flush=True)
                    elif event.key == pygame.K_2:
                        viz_mode = 'seeds'
                        print("Visualization mode: Seed segments", flush=True)
                    elif event.key == pygame.K_3:
                        viz_mode = 'lines'
                        print("Visualization mode: Final line segments", flush=True)
                    elif event.key == pygame.K_4:
                        viz_mode = 'all'
                        print("Visualization mode: All", flush=True)
                
                if pygame.mouse.get_focused():
                    sensor_on = True 
                elif not pygame.mouse.get_focused():
                    sensor_on = False

            if sensor_on:
                position = pygame.mouse.get_pos()
                scanner.pos = tuple(position)
                sensor_output_data = scanner.sense()
                print(f"Sensor data points: {len(sensor_output_data)}", flush=True)
                
                if sensor_output_data:
                    # Clear previous visualization
                    world.map.fill((0, 0, 0))
                    world.clear_point_cloud()
                    
                    # Transform polar data to point cloud
                    laser_points = transform_polar_point_set(sensor_output_data)
                    world.point_cloud = [p[0] for p in laser_points]
                    
                    # Visualize raw point cloud
                    if viz_mode in ['points', 'all']:
                        world.visualize_point_cloud()
                    
                    # Set laser points for extraction
                    line_extractor.set_laser_points(laser_points)
                    
                    # Extract line segments using seeded region growing
                    break_point = 0
                    seed_segments = []
                    final_segments = []
                    
                    while break_point < line_extractor.NP - line_extractor.PMIN:
                        # Algorithm 1: Detect seed segment
                        seed_data = line_extractor.detect_seed_segments(position, break_point)
                        
                        if seed_data is None:
                            break_point += 1
                            continue
                        
                        points, pred_points, (i, j) = seed_data
                        seed_segments.append(seed_data)
                        
                        grown_segment = line_extractor.seed_segment_growing((i, j), break_point, position=position)
                        
                        if grown_segment is not None:
                            final_segments.append(line_extractor.line_segments[-1])
                            # uppdate break point to end of grown segment
                            break_point = grown_segment[3] + 1
                        else:
                            #  move past this seed if growth fails
                            break_point = j + 1
                    
                    if len(final_segments) > 1:
                        final_segments = line_extractor.process_overlap_regions()
                    
                    validated_segments = []
                    for seg in final_segments:
                        if line_extractor.validate_segment_continuity(seg, sensor_position=position):
                            validated_segments.append(seg)
                        else:
                            print(f"  Rejected segment spanning gap: length={seg.get('length', 0):.1f}, "
                                  f"points={seg.get('point_count', 0)}", flush=True)
                    
                    final_segments = validated_segments
                    
                    #  qssociate our observed segments with existing landmarks
                    associations = landmark_manager.associate_observations(final_segments, current_frame=frame_count)
                    
                    # remove old landmarks that haven't been seen recently : 
                    landmark_manager.remove_old_landmarks(max_age=10, current_frame=frame_count)
                    
                    #  strong landmarks(features observed multiple timess)
                    strong_landmarks = landmark_manager.get_strong_landmarks()
                    print("strong langmarks:",strong_landmarks)
                    # Visualize seed segments (for debugging)
                    if viz_mode in ['seeds', 'all']:
                        world.visualize_seed_segments(seed_segments, color=(255, 255, 0), thickness=1)
                    
                    #draw the final line segments (current observations)
                    if viz_mode in ['lines', 'all']:
                        world.visualize_line_segments(final_segments, color=(255, 0, 0), thickness=2)
                    
                    
                    if strong_landmarks:
                        world.visualize_landmarks(strong_landmarks, color=(0, 255, 255), thickness=3)
                    
                    
                    print(f"Frame {frame_count}: {len(seed_segments)} seeds, {len(final_segments)} segments, "
                          f"{len(associations)} associations, {len(strong_landmarks)} strong landmarks", flush=True)
                    
                    frame_count += 1
            
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
