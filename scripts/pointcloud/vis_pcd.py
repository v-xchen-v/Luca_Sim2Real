
import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
import open3d as o3d
import numpy as np
from pointcloud_processing.transformations import _make_point_cloud_homogeneous, _make_point_cloud_xyz_array
    
def _show_point_cloud_window(point_cloud: o3d.geometry.PointCloud):
    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    opt = viewer.get_render_option()
    pcd = o3d.geometry.PointCloud()
    vtx = np.asanyarray( point_cloud.points)  # XYZ
    flipy_points = vtx
    # flipy_points[:, 1] *= -1 
    # flipy_points[:, 2] *= -1 
    # # flipy_points[:, 0] *= 0.001   
    R_x_90 = np.array([[1, 0, 0],
                   [0, 0, -1],
                   [0, 1, 0]])
    R_y_90 = np.array([[0, 0, 1],
                   [0, 1, 0],
                   [-1, 0, 0]])
    R_z_90 = np.array([[0, -1, 0],
                   [1, 0, 0],
                   [0, 0, 1]])
    R = np.array([[1, 0, 0],
                    [0, -1, 0],  # Flip Y-axis
                    [0, 0, -1]])  # Flip Z-axis
    flipy_points = flipy_points @ R.T
    T_depth_to_rgb = np.load('calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy')
    # T_rot180_x = np.eye(4)
    # T_rot180_x[1, 1] = -1
    # T_rot180_x[2, 2] = -1
    # flipy_points = _make_point_cloud_xyz_array(np.array([
    #                  T_depth_to_rgb @ T_rot180_x @ p 
    # #                  for p in _make_point_cloud_homogeneous(flipy_points)]))
    # flipy_points = _make_point_cloud_xyz_array(np.array([
    #                  T_depth_to_rgb  @ p 
    #                  for p in _make_point_cloud_homogeneous(flipy_points)]))
    # flipy_points = flipy_points @ R_y_90.T 
    pcd.points = o3d.utility.Vector3dVector(vtx)
    # viewer.add_geometry(pcd)
    
    # draw the point cloud with texture color
    pcd.colors = point_cloud.colors
    viewer.add_geometry(pcd)
    opt.show_coordinate_frame = True
    opt.background_color = np.asarray([0, 0, 0])
    viewer.run()
    viewer.destroy_window()
        
# Load and visualize the saved point cloud
# pcd = o3d.io.read_point_cloud("data/scene_data/test_scene_data/test_scene.pcd")
pcd = o3d.io.read_point_cloud("/home/yichao/Documents/repos/Luca_Transformation/data/scene_data/test_scene_data/test_scene_filtered_point_cloud.npy")
# o3d.visualization.draw_geometries([pcd])
# _show_point_cloud_window(pcd)
# Create a coordinate frame centered at the origin
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

# Visualize the reloaded point cloud with coordinate axes
o3d.visualization.draw_geometries([pcd, axes])