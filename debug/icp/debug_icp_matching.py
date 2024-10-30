import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

import open3d as o3d
from pointcloud_processing.icp_matching import align_source_to_target
import numpy as np
from pointcloud_processing.visualization_utils import show_points_in_window

def numpy_to_o3d(points: np.ndarray) -> o3d.geometry.PointCloud:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def get_points_center(points: np.ndarray) -> np.ndarray:
    return np.mean(points, axis=0)

def get_point_cloud_center(point_cloud: o3d.geometry.PointCloud) -> np.ndarray:
    return np.mean(np.asarray(point_cloud.points), axis=0)

# Example usage
if __name__ == "__main__":
    # Load point clouds (replace with your paths)
    target = np.load("/home/yichao/Documents/repos/Luca_Transformation/data/scene_data/orange_test_scene_data/test_scene_filtered_point_cloud.npy")
    target_pc = numpy_to_o3d(target)
    source = np.load("data/pointcloud_data/candidiate_objects/orange.npy")
    source_pc = numpy_to_o3d(source)

    
    print(f'center before align: {get_point_cloud_center(source_pc)}')
    print(f'center before align: {get_point_cloud_center(target_pc)}')
    
    # Align and restore point clouds
    aligned_source, restored_target = align_source_to_target(source_pc, target_pc)

    print(f'center after align: {get_point_cloud_center(aligned_source)}')
    print(f'center after align: {get_point_cloud_center(restored_target)}')
    
    # Visualize the result
    # o3d.visualization.draw_geometries([aligned_source, restored_target])
    
# Example of orange
# center before align: [-0.00043384  0.00034562 -0.00473993]
# center before align: [-0.09760619 -0.00890814 -0.04871982]

# center after align: [-0.10564639  0.0024461  -0.03722779]
# center after align: [-0.09760619 -0.00890814 -0.04871982]