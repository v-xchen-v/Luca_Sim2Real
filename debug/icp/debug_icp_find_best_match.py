import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


import open3d as o3d
import glob
from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud
from pointcloud_processing.icp_matching import get_closest_pcd_match

def find_best_matching_pcd(target_pcd_path, candidate_pcd_folder, max_correspondence_distance=0.1,w_fit=1.0, w_rmse=100.0):
    # Load the target point cloud
    target_pcd = load_npy_file_as_point_cloud(target_pcd_path)
    
    best_pcd = None
    best_fitness = 0
    best_score = -float('inf')
    best_rmse = float('inf')
    best_transformation = None
    
    # Load all candidate point clouds
    candidate_files = glob.glob(f"{candidate_pcd_folder}/*.npy")

    candidate_pcds = []
    for candidate_file in candidate_files:
        print("Processing:", candidate_file)
        candidate_pcd = load_npy_file_as_point_cloud(candidate_file)
        candidate_pcds.append(candidate_pcd)
    
    # Find the best matching point cloud
    best_pcd, best_pcd_index, best_fitness, best_rmse, best_transformation, best_score = \
        get_closest_pcd_match(target_pcd, candidate_pcds, 
                              max_correspondence_distance, w_fit, w_rmse)

    return best_pcd, best_pcd_index, best_fitness, best_rmse, best_transformation, best_score

# Specify paths
target_pcd_path = "data/scene_data/bottle_coconut_test_scene_data/test_scene_filtered_point_cloud.npy" #"path/to/target.pcd"
candidate_pcd_folder = "data/pointcloud_data/candidiate_objects"

# Find the best matching file
best_pcd, best_pcd_index, best_fitness, best_rmse, best_transformation, best_score = find_best_matching_pcd(target_pcd_path, candidate_pcd_folder)

best_pcd_file = glob.glob(f"{candidate_pcd_folder}/*.npy")[best_pcd_index]
print("Best Matching PCD File:", best_pcd_file)
print("Best Fitness:", best_fitness)
print("Best Transformation Matrix:\n", best_transformation)
print("Best RMSE:", best_rmse)
print("Best Score:", best_score)
