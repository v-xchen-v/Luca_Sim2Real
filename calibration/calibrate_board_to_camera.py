
from .checkerboard_utils import detect_checkerboard_corners, save_visualization, generate_object_points, compute_reprojection_error
import cv2
import numpy as np
import os

def compute_table_to_camera(image, pattern_size, square_size, mtx, dist, report_dir, error_threshold):
    """
    Compute the transformation matrix from the calibration board (table) to the camera,
    and raise an error if the reprojection error exceeds the threshold.

    Parameters:
    - image: Input image (NumPy array).
    - pattern_size: Tuple indicating the checkerboard size (rows, columns).
    - square_size: Size of a square on the checkerboard (in meters).
    - K: Camera intrinsic matrix.
    - dist: Distortion coefficients.
    - report_dir: Directory to save the visualization image and error report.
    - error_threshold: Maximum allowable reprojection error.

    Returns:
    - T: 4x4 transformation matrix (table to camera).
    """
    ret, corners = detect_checkerboard_corners(image, pattern_size)

    if not ret:
        raise ValueError("Checkerboard not found in the image.")

    obj_points = generate_object_points(pattern_size, square_size)

    # Compute rotation and translation vectors using solvePnP
    _, rvec, tvec = cv2.solvePnP(obj_points, corners, mtx, dist)

    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Construct the 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    
    # Compute the reprojection error
    error = compute_reprojection_error(obj_points, corners, rvec, tvec, mtx, dist)

    # Raise an error if the reprojection error exceeds the threshold
    if error > error_threshold:
        raise ValueError(f"Reprojection error {error:.4f} exceeds threshold {error_threshold:.4f}")

    if report_dir is not None:
        if not os.path.exists(report_dir):
            os.makedirs(report_dir)
        
        # Save the corner visualization
        vis_image_path = f"{report_dir}/corner_visualization.jpg"
        save_visualization(image, corners, pattern_size, ret, vis_image_path)
    
        # Save the transformation matrix to a .npy file
        T_path = f"{report_dir}/table_to_camera.npy"
        np.save(T_path, T)
        print(f"Transformation matrix saved to {T_path}.")
        
        # Save the reprojection error to a text file
        error_report_path = f"{report_dir}/reprojection_error.txt"
        with open(error_report_path, "w") as f:
            f.write(f"Reprojection Error: {error:.4f}\n")
        print(f"Reprojection error saved to {error_report_path}.")
        
    return T