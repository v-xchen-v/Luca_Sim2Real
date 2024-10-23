"""Once the match is found, transforms the object’s frame to the world frame using the appropriate transformation matrix."""

from pytransform3d.transformations import transform
import numpy as np

def _make_point_cloud_homogeneous(points):
    """Appends 1 to the points to make them homogeneous.

    Args:
        points (np.ndarray): The input point cloud.

    Returns:
        np.ndarray: The homogeneous point cloud.
    """
    # check points_in_camera's shape should be (n, 3)
    if points.shape[1] != 3:
        raise ValueError("The input point cloud should have shape (n, 3).")
    
    return np.hstack((points, np.ones((points.shape[0], 1))))

def camera_to_calibration_board_frame(points_in_camera, T_camera_to_calibration_board):
    """Transforms the point cloud from the camera coordinate system to the world coordinate system.

    Args:
        points (np.ndarray): The input point cloud that captured points from the camera.
        T_camera_to_calibration_board (np.ndarray): The transformation matrix from the camera to the calibration board.

    Returns:
        np.ndarray: The transformed point cloud in calibration board coordinate system.
    """
    # make the points homogeneous
    points_in_camera = _make_point_cloud_homogeneous(points_in_camera)
    
    # transform the points from camera frame to calibration board frame
    points_in_calibration_board = transform(T_camera_to_calibration_board, points_in_camera)
    return points_in_calibration_board

def camera_to_table_frame(points_in_camera, T_camera_to_table):
    """Transforms the point cloud from the camera coordinate system to the world coordinate system.

    Args:
        points (np.ndarray): The input point cloud that captured points from the camera.
        T_camera_to_table (np.ndarray): The transformation matrix from the camera to the table.

    Returns:
        np.ndarray: The transformed point cloud in table coordinate system.
    """
    # make the points homogeneous
    points_in_table = _make_point_cloud_homogeneous(points_in_camera)
    
    # transform the points from camera frame to table frame
    points_in_table = transform(T_camera_to_table, points_in_camera)
    return points_in_table