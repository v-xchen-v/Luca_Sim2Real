
from .checkerboard_utils import detect_checkerboard_corners, save_visualization, generate_object_points, compute_reprojection_error
import cv2
import numpy as np
import os
from .calibration_exceptions import CalibrationBoardNotFoundError, ReprojectionThresholdExceededError
from camera_operations.camera_capture import RealSenseCamera

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
    - error_threshold: Maximum allowable reprojection error. if None, no error is raised.

    Returns:
    - T: 4x4 transformation matrix (table to camera).
    """
    ret, corners = detect_checkerboard_corners(image, pattern_size)

    if not ret:
        # Define a new error CalibrationErrror
        
        raise CalibrationBoardNotFoundError("Checkerboard not found in the image.")

    obj_points = generate_object_points(pattern_size, square_size)

    # Compute rotation and translation vectors using solvePnP
    _, rvec, tvec = cv2.solvePnP(obj_points, corners, mtx, dist)

    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Construct the 4x4 transformation matrix
    T_camera_to_object = np.eye(4)
    T_camera_to_object[:3, :3] = R
    T_camera_to_object[:3, 3] = tvec.flatten()
    
    T_object_to_camera = np.linalg.inv(T_camera_to_object)
    
    # Compute the reprojection error
    error = compute_reprojection_error(obj_points, corners, rvec, tvec, mtx, dist)

    # Raise an error if the reprojection error exceeds the threshold
    if error_threshold is not None and error > error_threshold:
        raise ReprojectionThresholdExceededError(f"Reprojection error {error:.4f} exceeds threshold {error_threshold:.4f}")

    if report_dir is not None:
        if not os.path.exists(report_dir):
            os.makedirs(report_dir)
        
        # Save the corner visualization
        vis_image_path = f"{report_dir}/corner_visualization.jpg"
        save_visualization(image, corners, pattern_size, ret, vis_image_path)
        
        # read the image again to draw the axes
        vis_image = cv2.imread(vis_image_path)
        # draw axes on the first corner
        axis_length = 0.03
        axis_points = np.array([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
        projected_axis_points, _ = cv2.projectPoints(axis_points, rvec, tvec, mtx, dist)
        projected_axis_points = projected_axis_points.astype(int)
        cv2.line(vis_image, tuple(corners[0].flatten().astype(int)), tuple(projected_axis_points[1].flatten()), (0, 0, 255), 2)
        cv2.line(vis_image, tuple(corners[0].flatten().astype(int)), tuple(projected_axis_points[2].flatten()), (0, 255, 0), 2)
        cv2.line(vis_image, tuple(corners[0].flatten().astype(int)), tuple(projected_axis_points[3].flatten()), (255, 0, 0), 2)
        # save the image with axes
        cv2.imwrite(vis_image_path, vis_image)
    
        # Save the transformation matrix to a .npy file
        T_path = f"{report_dir}/table_to_camera.npy"
        np.save(T_path, T_object_to_camera)
        print(f"Transformation matrix saved to {T_path}.")
        
        # Save the reprojection error to a text file
        error_report_path = f"{report_dir}/reprojection_error.txt"
        with open(error_report_path, "w") as f:
            f.write(f"Reprojection Error: {error:.4f}\n")
        print(f"Reprojection error saved to {error_report_path}.")
        
    return T_object_to_camera

def capture_frame_and_save_table_calibration(pattern_size, square_size, mtx, dist, report_dir, error_threshold):
    camera = RealSenseCamera()
    
    while True:
        try:
            # Get the RGB frame
            frame = camera.get_rgb_frame()
            
            try:
                T_table_to_camera = compute_table_to_camera(frame, pattern_size, square_size, mtx, dist, report_dir, error_threshold)
                return T_table_to_camera
            except ReprojectionThresholdExceededError as e:
                print(e)
                continue
            except CalibrationBoardNotFoundError as e:
                print(e)
                continue
            
        
        except RuntimeError as e:
            print(e)
            break

    # Release resources
    camera.release()