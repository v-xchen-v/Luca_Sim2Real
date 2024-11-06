"""Once the match is found, transforms the object’s frame to the world frame using the appropriate transformation matrix."""

from pytransform3d.transformations import transform
import numpy as np
from pytransform3d.transformations import invert_transform, transform
import open3d as o3d

# def _make_point_cloud_homogeneous(points):
#     """Appends 1 to the points to make them homogeneous.

#     Args:
#         points (np.ndarray): The input point cloud.

#     Returns:
#         np.ndarray: The homogeneous point cloud.
#     """
#     # check points_in_camera's shape should be (n, 3)
#     if points.shape[1] != 3:
#         raise ValueError("The input point cloud should have shape (n, 3).")
    
#     return np.hstack((points, np.ones((points.shape[0], 1))))

# def _make_point_cloud_xyz_array(points):
#     """Extracts the xyz coordinates from the homogeneous points.

#     Args:
#         points (np.ndarray): The input point cloud.

#     Returns:
#         np.ndarray: The xyz coordinates.
#     """
#     return np.array(points)[:, :3]

# def camera_to_calibration_board_frame(points_in_camera: np.ndarray, T_camera_to_calibration_board):
#     """Transforms the point cloud from the camera coordinate system to the world coordinate system.

#     Args:
#         points (np.ndarray): The input point cloud that captured points from the camera.
#         T_camera_to_calibration_board (np.ndarray): The transformation matrix from the camera to the calibration board.

#     Returns:
#         np.ndarray: The transformed point cloud in calibration board coordinate system.
#     """
#     # make the points homogeneous
#     points_in_camera = _make_point_cloud_homogeneous(points_in_camera)
    
#     # transform the points from camera frame to calibration board frame
#     points_in_calibration_board = transform(T_camera_to_calibration_board, points_in_camera)
#     return points_in_calibration_board

# def transform_pointcloud_from_depthcam_to_rgbcam(points_in_depthcam, T_depthcam_to_rgbcam):
#     """Transforms the point cloud from the camera coordinate system to the world coordinate system.

#     Args:
#         points (np.ndarray): The input point cloud that captured points from the camera.
#         T_camera_to_table (np.ndarray): The transformation matrix from the camera to the table.

#     Returns:
#         np.ndarray: The transformed point cloud in table coordinate system.
#     """
#     point_is_homo = True
#     if points_in_depthcam.shape[1] == 3:
#         # The input point cloud should have shape (n, 3).
#         point_is_homo = False
        
#     if not point_is_homo:        
#         # make the points homogeneous
#         points_in_depthcam = _make_point_cloud_homogeneous(points_in_depthcam)
    
#     # transform the points from camera frame to table frame
#     points_in_rgbcam = transform(T_depthcam_to_rgbcam, points_in_depthcam)
    
#     if not point_is_homo:
#         points_in_rgbcam = _make_point_cloud_xyz_array(points_in_rgbcam)
#     return points_in_rgbcam

# def transform_point_cloud(point_cloud, T_source_to_target):
#     """
#     Transforms a point cloud from the camera frame to the board (table) frame.

#     Parameters:
#         point_cloud (numpy.ndarray): N x 3 array of points in camera coordinates.
#         T_camera_to_board (numpy.ndarray): 4 x 4 transformation matrix.

#     Returns:
#         numpy.ndarray: N x 3 array of points in table coordinates.
#     """
#     # Add a column of ones to convert the points to homogeneous coordinates (N x 4)
#     num_points = point_cloud.shape[0]
#     homogeneous_points = np.hstack((point_cloud, np.ones((num_points, 1))))

#     # Apply the transformation matrix
#     transformed_homogeneous = homogeneous_points @ T_source_to_target.T

#     # Convert back to 3D points by removing the homogeneous coordinate
#     transformed_points = transformed_homogeneous[:, :3] / transformed_homogeneous[:, 3:]

#     return transformed_points


def transform_point_cloud_from_camera_to_table(pc_in_depthcam, T_rgbcam_to_table, T_depthcam_to_rgbcam) -> o3d.geometry.PointCloud:
    """Transforms the point cloud from the camera coordinate system to the world coordinate system.

    Args:
        points (np.ndarray): The input point cloud that captured points from the camera.
        T_camera_to_table (np.ndarray): The transformation matrix from the camera to the table.

    Returns:
        np.ndarray: The transformed point cloud in table coordinate system.
    """
    import open3d as o3d
    # pc = o3d.geometry.PointCloud()
    # pc.points = o3d.utility.Vector3dVector(pc_in_depthcam)
    pc_in_depthcam.transform(invert_transform(T_rgbcam_to_table @ T_depthcam_to_rgbcam))
    
    # for debugging purpose
    if False:
        # Visualize the transformed point cloud
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        o3d.visualization.draw_geometries(
                                [pc_in_depthcam, axes],
                                window_name='Transformed Point Clou',
                                width=800, height=600,
                                left=0, top=1)

    # Optionally save the transformed PCD to a new file
    # o3d.io.write_point_cloud("transformed_pointcloud.pcd", pcd)

    # points_in_depthcam[:, 1]*=-1
    
    # make the points homogeneous
    # points_in_depthcam = _make_point_cloud_homogeneous(points_in_depthcam)
    
    # transform the points from camera frame to table frame
    # point_in_rgbcam = transform_pointcloud_from_depthcam_to_rgbcam(points_in_depthcam, T_depthcam_to_rgbcam)
    
    # a 4x4 matrix of rotate 180 degree around x axis
    # o3d -> realsense
    # T_rot180_x = np.eye(4)
    # T_rot180_x[1, 1] = -1 # flip y
    # T_rot180_x[2, 2] = -1 # flip z
    
    # # point_in_rgbcam_rotated = transform(T_rot180_x, points_in_depthcam)
    # # points_in_table = transform(T_rgbcam_to_table, points_in_depthcam)
    # # points_in_table = [T_rgbcam_to_table@T_depthcam_to_rgbcam@p for p in points_in_depthcam]
    # # points_in_table = [T_rgbcam_to_table@p for p in points_in_depthcam]
    # print(f'depthcam_to_rgbcam translation: {T_depthcam_to_rgbcam[:3, 3]}')
    # from scipy.spatial.transform import Rotation as R   
    # print(f'depthcam_to_rgbcam rotation: {R.from_matrix(T_depthcam_to_rgbcam[:3, :3]).as_euler("xyz", degrees=True)}')
    # # points_in_table = [invert_transform(T_rot180_x) @
    # #                    T_rgbcam_to_table@ T_depthcam_to_rgbcam @ T_rot180_x@p for p in points_in_depthcam]
    
    # # points_in_table = transform(
    # #                         invert_transform(T_rot180_x) @ T_rgbcam_to_table@ invert_transform(T_depthcam_to_rgbcam) @ T_rot180_x,
    # #                         points_in_depthcam)
    
    # # points_in_table = transform(
    # #                     invert_transform(T_rot180_x) @ T_rgbcam_to_table @ T_rot180_x,
    # #                     points_in_depthcam)
    
    # points_in_table = transform(
    #                     invert_transform(T_rot180_x) @ T_rgbcam_to_table@ T_depthcam_to_rgbcam @ T_rot180_x,
    #                     pc_in_depthcam)
    
    # # points_in_table = transform_point_cloud(points_in_depthcam,
    # #                     invert_transform(T_rot180_x) @ T_rgbcam_to_table @ T_rot180_x,
    # #                     )
    # points_in_table = _make_point_cloud_xyz_array(points_in_table)
    # # T_rgbcam_to_table[:3, 1]*=-1
    # p_tables = []
    # for one_point in points_in_depthcam:
    #     one_point = np.append(one_point, 1)
    #     p_table = (T_rgbcam_to_table@T_depthcam_to_rgbcam@T_rot180_x@one_point)[:3]
    #     p_tables.append(p_table)
    return pc_in_depthcam