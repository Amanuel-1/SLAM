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
LANDMARK_DETECTION_THRESHOLD = 1.0 
DATA_ASSOCIATION_THRESHOLD = 20.0  # Similarity score threshold for line segment matching (lower = stricter)
                                    # Based on weighted combination of midpoint distance, line distance, and length
MAX_LANDMARK_DISTANCE = 150.0

# Line extraction parameters (seeded region growing algorithm)
LINE_EXTRACTION_EPSILON = 2.0
LINE_EXTRACTION_DELTA = 2.0
LINE_EXTRACTION_S_NUM = 5
LINE_EXTRACTION_P_MIN = 6
LINE_EXTRACTION_L_MIN = 40.0
LINE_EXTRACTION_G_MAX = 8.0
LINE_EXTRACTION_ANGLE_GAP_MAX = 0.08
LINE_EXTRACTION_DISTANCE_JUMP_RATIO = 1.3

# Visualization
BACKGROUND_COLOR = (255, 255, 255)
OBSTACLE_COLOR = (0, 0, 0)
ROBOT_COLOR = (0, 0, 255)
ESTIMATED_ROBOT_COLOR = (255, 0, 0)
LANDMARK_COLOR = (0, 255, 0)
ESTIMATED_LANDMARK_COLOR = (255, 165, 0)
LINE_LANDMARK_COLOR = (0, 255, 255)  # Cyan for line segment landmarks
