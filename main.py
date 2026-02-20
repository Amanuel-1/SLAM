"""Main entry point for SLAM simulation"""
import pygame
import numpy as np
from world import World
from robot import Robot
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
        pygame.init()
        print("Pygame initialized", flush=True)
        
        print("Creating world...", flush=True)
        world = World(map_path="world/maps/indoor1.png")
        
        original_map = world.map.copy()

        world.map.fill((0,0,0))
        
        print("Creating robot...", flush=True)
        robot = Robot(x=world.width//2, y=world.height//2, theta=0)
        world.robot = robot
        
        scanner = Lidar(original_map, robot=robot)
        robot.add_sensor(scanner)
        
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
        
        landmark_manager = LineLandmarkManager(
            association_threshold=config.DATA_ASSOCIATION_THRESHOLD,
            min_landmark_length=config.LINE_EXTRACTION_L_MIN,
            min_observations=2
        )
        
        frame_count = 0
        viz_mode = 'all'
        show_robot_path = True
        show_robot = True
        
        robot_positioned = False
        positioning_mode = True
        
        running = True
        dt = 1.0 / config.FPS
        clock = pygame.time.Clock()
        
        print("Starting main loop...", flush=True)
        print("=" * 60)
        print("POSITIONING MODE: Click anywhere to place the robot")
        print("=" * 60)
        
        # initial positioning - let user click where to start
        while running and positioning_mode:
            world.map.fill((0, 0, 0))
            
            font = pygame.font.Font(None, 36)
            text = font.render("Click to position robot", True, (255, 255, 255))
            text_rect = text.get_rect(center=(world.width // 2, 50))
            world.map.blit(text, text_rect)
            
            mouse_pos = pygame.mouse.get_pos()
            pygame.draw.circle(world.map, (0, 255, 0), mouse_pos, 15, 2)
            pygame.draw.line(world.map, (255, 255, 0), mouse_pos, 
                           (mouse_pos[0] + 30, mouse_pos[1]), 3)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    positioning_mode = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    robot.pose.x = mouse_pos[0]
                    robot.pose.y = mouse_pos[1]
                    robot.pose.theta = 0
                    robot.true_pose.x = mouse_pos[0]
                    robot.true_pose.y = mouse_pos[1]
                    robot.true_pose.theta = 0
                    positioning_mode = False
                    robot_positioned = True
                    print(f"Robot positioned at ({mouse_pos[0]}, {mouse_pos[1]})")
                    print("=" * 60)
                    print("Controls: WASD or Arrow keys to move, Q/E for combined movement")
                    print("Press 1-4 to change visualization, 5 to toggle path, R to reset path")
                    print("Press P to reposition robot at any time")
                    print("=" * 60)
            
            pygame.display.update()
            clock.tick(config.FPS)
        
        if not running:
            pygame.quit()
            return
        
        # main simulation loop
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
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
                    elif event.key == pygame.K_5:
                        show_robot_path = not show_robot_path
                        print(f"Robot path: {'ON' if show_robot_path else 'OFF'}", flush=True)
                    elif event.key == pygame.K_r:
                        robot.path.clear()
                        print("Path reset", flush=True)
                    elif event.key == pygame.K_p:
                        positioning_mode = True
                        robot.set_velocity(0, 0)
                        print("=" * 60)
                        print("POSITIONING MODE: Click to reposition robot")
                        print("=" * 60)
                elif event.type == pygame.MOUSEBUTTONDOWN and positioning_mode:
                    mouse_pos = pygame.mouse.get_pos()
                    robot.pose.x = mouse_pos[0]
                    robot.pose.y = mouse_pos[1]
                    robot.pose.theta = 0
                    robot.true_pose.x = mouse_pos[0]
                    robot.true_pose.y = mouse_pos[1]
                    robot.true_pose.theta = 0
                    robot.path.clear()
                    positioning_mode = False
                    print(f"Robot repositioned at ({mouse_pos[0]}, {mouse_pos[1]})")
                    print("Resumed - use WASD to move")
            
            # if we're in positioning mode, show preview and wait for click
            if positioning_mode:
                world.map.fill((0, 0, 0))
                
                font = pygame.font.Font(None, 36)
                text = font.render("Click to position robot (P to cancel)", True, (255, 255, 255))
                text_rect = text.get_rect(center=(world.width // 2, 50))
                world.map.blit(text, text_rect)
                
                mouse_pos = pygame.mouse.get_pos()
                pygame.draw.circle(world.map, (0, 255, 0), mouse_pos, 15, 2)
                pygame.draw.line(world.map, (255, 255, 0), mouse_pos, 
                               (mouse_pos[0] + 30, mouse_pos[1]), 3)
                
                keys = pygame.key.get_pressed()
                if keys[pygame.K_p]:
                    positioning_mode = False
                    print("Positioning cancelled")
                
                pygame.display.update()
                clock.tick(config.FPS)
                continue
            
            # handle keyboard controls
            keys = pygame.key.get_pressed()
            
            v = 0.0
            omega = 0.0
            
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                v = config.MAX_VELOCITY * config.KEYBOARD_VELOCITY_SCALE
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                v = -config.MAX_VELOCITY * config.KEYBOARD_VELOCITY_SCALE
            
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                omega = config.MAX_ANGULAR_VELOCITY * config.KEYBOARD_ANGULAR_SCALE
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                omega = -config.MAX_ANGULAR_VELOCITY * config.KEYBOARD_ANGULAR_SCALE
            
            if keys[pygame.K_q]:
                v = config.MAX_VELOCITY * config.KEYBOARD_VELOCITY_SCALE
                omega = config.MAX_ANGULAR_VELOCITY * config.KEYBOARD_ANGULAR_SCALE * 0.5
            elif keys[pygame.K_e]:
                v = config.MAX_VELOCITY * config.KEYBOARD_VELOCITY_SCALE
                omega = -config.MAX_ANGULAR_VELOCITY * config.KEYBOARD_ANGULAR_SCALE * 0.5
            
            robot.set_velocity(v, omega)
            robot.update(dt, original_map)
            
            sensor_output_data = scanner.sense()
            
            # clear screen and draw path first
            world.map.fill((0, 0, 0))
            
            if show_robot_path:
                world.visualize_robot_path(robot.path, color=config.ROBOT_PATH_COLOR)
            
            if sensor_output_data:
                world.clear_point_cloud()
                
                laser_points = transform_polar_point_set(sensor_output_data)
                world.point_cloud = [p[0] for p in laser_points]
                
                if viz_mode in ['points', 'all']:
                    world.visualize_point_cloud()
                
                line_extractor.set_laser_points(laser_points)
                
                break_point = 0
                seed_segments = []
                final_segments = []
                
                position = (robot.pose.x, robot.pose.y)
                
                # extract line segments from point cloud
                while break_point < line_extractor.NP - line_extractor.PMIN:
                    seed_data = line_extractor.detect_seed_segments(position, break_point)
                    
                    if seed_data is None:
                        break_point += 1
                        continue
                    
                    points, pred_points, (i, j) = seed_data
                    seed_segments.append(seed_data)
                    
                    grown_segment = line_extractor.seed_segment_growing((i, j), break_point, position=position)
                    
                    if grown_segment is not None:
                        final_segments.append(line_extractor.line_segments[-1])
                        break_point = grown_segment[3] + 1
                    else:
                        break_point = j + 1
                
                if len(final_segments) > 1:
                    final_segments = line_extractor.process_overlap_regions()
                
                # filter out segments that span gaps
                validated_segments = []
                for seg in final_segments:
                    if line_extractor.validate_segment_continuity(seg, sensor_position=position):
                        validated_segments.append(seg)
                
                final_segments = validated_segments
                
                # track landmarks over time
                associations = landmark_manager.associate_observations(final_segments, current_frame=frame_count)
                landmark_manager.remove_old_landmarks(max_age=10, current_frame=frame_count)
                strong_landmarks = landmark_manager.get_strong_landmarks()
                
                # draw everything based on viz mode
                if viz_mode in ['seeds', 'all']:
                    world.visualize_seed_segments(seed_segments, color=(255, 255, 0), thickness=1)
                
                if viz_mode in ['lines', 'all']:
                    world.visualize_line_segments(final_segments, color=(255, 0, 0), thickness=2)
                
                if strong_landmarks:
                    world.visualize_landmarks(strong_landmarks, color=(0, 255, 255), thickness=3)
                
                if frame_count % 30 == 0:
                    print(f"Frame {frame_count}: {len(seed_segments)} seeds, {len(final_segments)} segments, "
                          f"{len(strong_landmarks)} landmarks | Pos: ({robot.pose.x:.1f}, {robot.pose.y:.1f})", flush=True)
                
                frame_count += 1
            
            # draw robot on top of everything
            if show_robot:
                world.visualize_robot(robot, color=config.ROBOT_COLOR)
            
            pygame.display.update()
            clock.tick(config.FPS)
        
        print("Exiting...", flush=True)
        pygame.quit()
        
    except Exception as e:
        print(f"ERROR: {e}", flush=True)
        traceback.print_exc()
        pygame.quit()
        sys.exit(1)


if __name__ == "__main__":
    main()
