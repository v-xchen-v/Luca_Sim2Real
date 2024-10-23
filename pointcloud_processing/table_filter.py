"""Filters out points that are not on the table."""

def filter_point_outside_operation_area(points_in_table, x_range, y_range, z_range):
    """Filters out points that are not on the table.

    Args:
        points_in_table (np.ndarray): The input point cloud in the table coordinate system.
        x_range (Tuple[int, int]): The range of the x-axis.
        y_range (Tuple[int, int]): The range of the y-axis.
        z_range (Tuple[int, int]): The range of the z-axis.

    Returns:
        np.ndarray: The filtered point cloud.
    """
    x_min, x_max = x_range
    y_min, y_max = y_range
    z_min, z_max = z_range
    
    # handle z_max is None
    if z_max is None:
        z_max = points_in_table[:, 2].max()
        
    # handle z_min is None
    if z_min is None:
        z_min = points_in_table[:, 2].min()

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
