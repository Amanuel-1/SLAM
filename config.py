import math
import numpy as np
"""Configuration constants for SLAM simulation"""

# World settings
WORLD_WIDTH = 1200
WORLD_HEIGHT = 700
MAP_RESOLUTION = 0.05  # meters per pixel
FPS = 60


ROBOT_RADIUS = 0.18  
MAX_VELOCITY = 0.5 
MAX_ANGULAR_VELOCITY = 1.5  # rad/s

# Lidar settings (long-range 2D laser scanner)
LIDAR_BEAMS = 360  
LIDAR_RANGE = 200.0 
LIDAR_FOV = 2 * math.pi 
LIDAR_NOISE_SIGMA = np.array([1.0, 0.05])
LIDAR_SCAN_RATE = 10 

# Odometry noise (realistic wheel encoder noise)
ODOM_NOISE_TRANSLATION = 0.05 
ODOM_NOISE_ROTATION = 0.03 

# SLAM settings (EKF-SLAM parameters)
LANDMARK_DETECTION_THRESHOLD = 2.0 
DATA_ASSOCIATION_THRESHOLD = 5.0  # Mahalanobis distance threshold for matching
MAX_LANDMARK_DISTANCE = 150.0

# Visualization
BACKGROUND_COLOR = (255, 255, 255)
OBSTACLE_COLOR = (0, 0, 0)
ROBOT_COLOR = (0, 0, 255)
ESTIMATED_ROBOT_COLOR = (255, 0, 0)
LANDMARK_COLOR = (0, 255, 0)
ESTIMATED_LANDMARK_COLOR = (255, 165, 0)
