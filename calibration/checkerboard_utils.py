"""
Checkerboard Detection:
    Uses OpenCV to detect the checkerboard corners from the input image.

Object Points Generation:
    Generates the 3D coordinates of the checkerboard corners in the board’s local frame.

Transformation Matrix Calculation:
    Uses cv2.solvePnP to compute the rotation and translation vectors between the board and the camera.
    Constructs a 4x4 transformation matrix from these vectors.

Reprojection Error:
    Calculates the reprojection error to assess the quality of the calibration. The error measures how closely the projected 3D points match the detected 2D points.
"""

import cv2
import numpy as np
from typing import Tuple
import os

def _find_chessboard_corners(gray: np.array, XX: int, YY:int, flags: int, criteria: Tuple[int, int ,float], winsize: Tuple[int, int]):
    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK
                                            +cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:
        # refining pixel coordinates for given 2d points
        corners2 = cv2.cornerSubPix(gray, corners, winsize, (-1, -1), criteria)
        return True, corners2
    else:
        return False, []
    
def detect_checkerboard_corners(image, pattern_size):
    """
    Detect checkerboard corners in an image.

    Parameters:
    - image: Input image (as a NumPy array).
    - pattern_size: Tuple indicating the number of inner corners (rows, columns).

    Returns:
    - ret: Boolean indicating if the checkerboard was found.
    - corners: Pixel coordinates of detected corners.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (YY, XX) = pattern_size
    ret, corners = _find_chessboard_corners(
        gray, XX, YY, 
        flags= cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK
                                                +cv2.CALIB_CB_NORMALIZE_IMAGE,
        criteria=(cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001), 
        winsize=(11, 11))
        
    # corners: order in row by row, and left to right in each row
    if not ret:
        print(f"Warning: Can not find checkerboard corners")
    return ret, corners

def generate_object_points(pattern_size: Tuple[int, int], square_size: float) -> np.array:
    """
    Generate 3D object points for the checkerboard.

    Parameters:
    - pattern_size: Tuple indicating the number of inner corners (rows, columns).
    - square_size: Size of a square on the checkerboard (in meters).

    Returns:
    - objp: Array of 3D object points.
    """
    

    # Initialize a zero array to store the 3D points for the checkboard corners
    (YY, XX) = pattern_size
    objp = np.zeros((XX * YY, 3), np.float32)
    # Set the x and y coordinates for each corner on the checkboard (z-coordinates are zero)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    # Scale the object points by the size of each square (L)
    L = square_size
    objp *= L
    
    return objp

def draw_detected_corners(image, corners, pattern_size, found):
    """
    Visualize detected corners on the checkerboard image.

    Parameters:
    - image: Input image (as a NumPy array).
    - corners: Pixel coordinates of detected corners.
    - pattern_size: Tuple indicating the checkerboard size (rows, columns).
    - found: Boolean indicating if the checkerboard was found.
    """
    vis_image = image.copy()  # Make a copy to draw on
    if found:
        # draw the first corner
        cv2.circle(vis_image, tuple(corners[0].flatten().astype(int)), 5, (0, 0, 255), -1)
        
        cv2.drawChessboardCorners(vis_image, pattern_size, corners, found)
        return vis_image
    else:
        print("Checkerboard not found. Cannot visualize corners.")
        return None
              
def save_visualization(image, corners, pattern_size, found, output_path):
    """
    Save the visualization of detected corners as an image.

    Parameters:
    - image: Input image (as a NumPy array).
    - corners: Pixel coordinates of detected corners.
    - pattern_size: Tuple indicating the checkerboard size (rows, columns).
    - found: Boolean indicating if the checkerboard was found.
    - output_path: Path to save the visualization image.
    """
    if found:
        vis_image = draw_detected_corners(image, corners, pattern_size, found)
        cv2.imwrite(output_path, vis_image)
        print(f"Corner visualization saved to {output_path}.")
    else:
        print("Checkerboard not found. No visualization saved.")
    
def compute_reprojection_error(obj_points, img_points, rvec, tvec, mtx, dist):
    """
    Compute the reprojection error to assess calibration quality.

    Parameters:
    - obj_points: 3D object points.
    - img_points: 2D image points (detected corners).
    - rvec: Rotation vector.
    - tvec: Translation vector.
    - mtx: Camera intrinsic matrix.
    - dist: Distortion coefficients.

    Returns:
    - error: The reprojection error value.
    """
    projected_img_points, _ = cv2.projectPoints(obj_points, rvec, tvec, mtx, dist)
    # error = np.sqrt(np.mean(np.sum((img_points - projected_img_points) ** 2, axis=2))) # Root Mean Square Error (RMSE), Outliers	More sensitive to large deviations (outliers)
    error = cv2.norm(img_points, projected_img_points, cv2.NORM_L2)/len(img_points) # Mean L2 Norm Error, 
    return error