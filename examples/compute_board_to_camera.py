"""
Given a image of a calibration board (table), this function computes the transformation matrix from the calibration board to the camera. 

- It uses the OpenCV function `cv2.solvePnP` to estimate the rotation and translation vectors, and then constructs a 4x4 transformation matrix from these vectors. 
- The function also computes the reprojection error and saves it to a text file. 
- If the reprojection error exceeds a specified threshold, an error is raised. The function returns the 4x4 transformation matrix.
"""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
    
from calibration.calibrate_board_to_camera import compute_table_to_camera
from calibration.calibration_data_loader import load_camera_intrinsic_from_npy
import cv2
from calibration.visualize_table_to_camera import visualize_table_to_camera

# Load the camera intrinsic matrix and distortion coefficients
mtx, dist = load_camera_intrinsic_from_npy("calibration/calibration_data/camera1/camera_intrinsics")

# Load the calibration board image
image_name = "captured_frame_002"
image = cv2.imread(f"data/debug_data/calibration/{image_name}.jpg")

# Define the calibration board parameters
# pattern_size = (8, 11)  # Checkerboard size (rows, columns)
# square_size = 0.02  # Size of a square on the checkerboard (in meters)
pattern_size = (5, 8)  # Checkerboard size (rows, columns)
square_size = 0.03  # Size of a square on the checkerboard (in meters)

# Define the error threshold for the reprojection error
error_threshold = 0.5

# Define the directory to save the visualization image and error report
report_dir = f"data/debug_data/calibration/output/{image_name}"

# Compute the transformation matrix from the calibration board to the camera
T_table_to_camera = compute_table_to_camera(image, pattern_size, square_size, mtx, dist, report_dir, error_threshold=None)

# Visualize the transformation
visualize_table_to_camera(T_table_to_camera)
