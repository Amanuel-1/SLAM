import math
import numpy as np
"""Configuration constants for SLAM simulation"""

# World settings
WORLD_WIDTH = 2500  # Optimized for 27" 2K display (2560x1440)
WORLD_HEIGHT = 1380
MAP_RESOLUTION = 0.05  # meters per pixel
FPS = 60


ROBOT_RADIUS = 0.18  
MAX_VELOCITY = 0.5 
MAX_ANGULAR_VELOCITY = 1.5  # rad/s

# Lidar settings (long-range 2D laser scanner)
LIDAR_BEAMS = 420  # Increased from 360 for denser point cloud (1 degree resolution)
LIDAR_RANGE = 200.0 
LIDAR_FOV = 2 * math.pi 
# Reduced noise for cleaner point clouds - distance noise in pixels, angle noise in radians
LIDAR_NOISE_SIGMA = np.array([0.1, 0.001])  # Reduced from [1.0, 0.05] for less scattered points
LIDAR_ENABLE_NOISE = True  # Set to False to disable noise completely for testing
LIDAR_SCAN_RATE = 10 

# Odometry noise (realistic wheel encoder noise)
ODOM_NOISE_TRANSLATION = 0.05 
ODOM_NOISE_ROTATION = 0.03 

# SLAM settings (EKF-SLAM parameters)
LANDMARK_DETECTION_THRESHOLD = 2.0 
DATA_ASSOCIATION_THRESHOLD = 20.0  # Similarity score threshold for line segment matching (lower = stricter)
                                    # Based on weighted combination of midpoint distance, line distance, and length
MAX_LANDMARK_DISTANCE = 150.0

# Line extraction parameters (seeded region growing algorithm)
# Based on: "A line segment extraction algorithm using laser data based on seeded region growing"
# Adjusted for wall detection: tighter thresholds to fit along walls, not across thickness
LINE_EXTRACTION_EPSILON = 2.0    # ε: Perpendicular distance threshold (d2) - pixels
                                  # Increased to allow fitting along longer walls
LINE_EXTRACTION_DELTA = 2.0      # δ: Point-to-predicted-point distance threshold (d1) - pixels
                                  # Increased to allow fitting along longer walls
LINE_EXTRACTION_S_NUM = 5        # S_num: Number of points to fit seed segment
                                  # Increased for more robust seed detection
LINE_EXTRACTION_P_MIN = 5        # P_min: Minimum number of points for valid segment
                                  # Increased to filter out short spurious segments
LINE_EXTRACTION_L_MIN = 30.0     # L_min: Minimum segment length (pixels)
                                  # Increased to prefer longer wall segments
LINE_EXTRACTION_G_MAX = 5.0      # G_max: Maximum gap between consecutive points (pixels)
                                  # Reduced to prevent bridging windows/doors
LINE_EXTRACTION_ANGLE_GAP_MAX = 0.1  # Maximum angular gap (radians) ~5.7 degrees
                                      # Prevents connecting across windows/doors
LINE_EXTRACTION_DISTANCE_JUMP_RATIO = 1.5  # Max ratio of distance change between consecutive points
                                            # Prevents connecting across gaps

# Visualization
BACKGROUND_COLOR = (255, 255, 255)
OBSTACLE_COLOR = (0, 0, 0)
ROBOT_COLOR = (0, 0, 255)
ESTIMATED_ROBOT_COLOR = (255, 0, 0)
LANDMARK_COLOR = (0, 255, 0)
ESTIMATED_LANDMARK_COLOR = (255, 165, 0)
LINE_LANDMARK_COLOR = (0, 255, 255)  # Cyan for line segment landmarks
