import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
"""To get the filtered point cloud, we need to load the point cloud and apply the filters to it.
Pre-requesties:
- A Calibration board on the scene and known or computed the T_calibration_board_to_camera.
"""
    
from pointcloud_processing.pointcloud_io import (save_image_and_point_cloud_from_realsense, load_npy_as_point_cloud)
from calibration.calibrate_board_to_camera import compute_table_to_camera
from pointcloud_processing.table_filter import filter_point_outside_operation_area
from pointcloud_processing.transformations import transform_point_cloud_from_camera_to_table
from pytransform3d.transformations import invert_transform
from pointcloud_processing.pointcloud_visualization import visualize_numpy_as_point_cloud
import cv2
import numpy as np

# Save the point cloud from the realsense camera
save_image_and_point_cloud_from_realsense('./data/debug_data/pointcloud_data/camera_captures', 'test', overwrite_if_exists=True)

# load the point cloud
from pointcloud_processing.pointcloud_io import load_point_cloud_as_numpy
points = load_point_cloud_as_numpy('./data/debug_data/pointcloud_data/camera_captures/test.pcd')
print(points.shape)

# Compute the table to camera transformation
table_calibration_image_path = './data/debug_data/pointcloud_data/camera_captures/test.png'
table_calibration_image = cv2.imread(table_calibration_image_path)

# load calibration pre-computed
from calibration.calibration_precomputed_data_loader import load_camera_intrinsics_from_npy
mtx, dist = load_camera_intrinsics_from_npy('./calibration/calibration_data/camera1/camera_intrinsics')
T_calibration_board_to_camera = compute_table_to_camera(image=table_calibration_image, 
                                                        pattern_size=(5, 8), square_size=0.03,
                                                        mtx=mtx, dist=dist, 
                                                        report_dir='./data/debug_data/pointcloud_data/camera_captures', error_threshold=1)

points_in_calibration_board = \
    transform_point_cloud_from_camera_to_table(load_npy_as_point_cloud(points), 
                                                T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                T_rgbcam_to_table=invert_transform(T_calibration_board_to_camera))
from pointcloud_processing.pointcloud_visualization import visualize_point_cloud
visualize_point_cloud(points_in_calibration_board)

points_filtered, _ = filter_point_outside_operation_area(points_in_calibration_board, 
                                                      x_range=(-1, 0), y_range=(0, 1), z_range=(-0.5, 0))

# # visualize the filtered point cloud
# from pointcloud_processing.pointcloud_io import save_point_cloud
# save_point_cloud('./data/debug_data/pointcloud_data/camera_captures/test_filtered.ply', points_filtered)
# print("Filtered point cloud is saved to ./data/debug_data/pointcloud_data/camera_captures/test_filtered.ply")

# Visualize the filtered point cloud
visualize_numpy_as_point_cloud(points_filtered)