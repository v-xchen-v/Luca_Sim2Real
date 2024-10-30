"""Filters out points that are not on the table."""
import numpy as np

def filter_point_outside_operation_area(pcd_in_table, x_range, y_range, z_range):
    """Filters out points that are not on the table.

    Args:
        points_in_table (np.ndarray): The input point cloud in the table coordinate system.
        x_range (Tuple[int, int]): The range of the x-axis.
        y_range (Tuple[int, int]): The range of the y-axis.
        z_range (Tuple[int, int]): The range of the z-axis.

    Returns:
        np.ndarray: The filtered point cloud.
    """
    
    points_in_table = np.asarray(pcd_in_table.points)[:, :3] # Shape: (N, 6), N is the number of points, x, y, z, r, g, b
    
    x_min, x_max = x_range
    y_min, y_max = y_range
    z_min, z_max = z_range
        
    # Helper function to get min or max with fallback for NaN
    def get_bound(array, func, default):
        value = func(array)
        return value if not np.isnan(value) else default

    # Define default min and max float values
    min_default = np.finfo(np.float32).min
    max_default = np.finfo(np.float32).max

    # Set x, y, z ranges with appropriate fallbacks
    x_min = get_bound(points_in_table[:, 0], np.min, min_default) if x_min is None else x_min
    x_max = get_bound(points_in_table[:, 0], np.max, max_default) if x_max is None else x_max

    y_min = get_bound(points_in_table[:, 1], np.min, min_default) if y_min is None else y_min
    y_max = get_bound(points_in_table[:, 1], np.max, max_default) if y_max is None else y_max

    z_min = get_bound(points_in_table[:, 2], np.min, min_default) if z_min is None else z_min
    z_max = get_bound(points_in_table[:, 2], np.max, max_default) if z_max is None else z_max

        

    # Filter out points that are not on the table
    x_mask = (points_in_table[:, 0] >= x_min) & (points_in_table[:, 0] <= x_max)
    y_mask = (points_in_table[:, 1] >= y_min) & (points_in_table[:, 1] <= y_max)     
    z_mask = (points_in_table[:, 2] >= z_min) & (points_in_table[:, 2] <= z_max)
    mask = x_mask & y_mask & z_mask
    # mask = (points_in_table[:, 0] >= x_min) & (points_in_table[:, 0] <= x_max) & \
    #        (points_in_table[:, 1] >= y_min) & (points_in_table[:, 1] <= y_max) & \
    #        (points_in_table[:, 2] >= z_min) & (points_in_table[:, 2] <= z_max)
    return points_in_table[mask]


# def transform_point_cloud_from_camera_to_table(points, camera_to_table_transform):
#     """Transforms the point cloud from the camera coordinate system to the table coordinate system.

#     Args:
#         points (np.ndarray): The input point cloud.
#         camera_to_table_transform (np.ndarray): The transformation matrix from the camera to the table.

#     Returns:
#         np.ndarray: The transformed point cloud.
#     """
#     return np.dot(camera_to_table_transform, points.T).T
