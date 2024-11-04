import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


import open3d as o3d
import numpy as np
import glob
from pointcloud_processing.icp_matching import icp_align
from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud

def find_best_matching_pcd(target_pcd_path, candidate_pcd_folder, max_correspondence_distance=0.1,w_fit=1.0, w_rmse=100.0):
    # Load the target point cloud
    target_pcd = load_npy_file_as_point_cloud(target_pcd_path)
    
    best_pcd_file = None
    best_fitness = 0
    best_score = -float('inf')
    best_rmse = float('inf')
    best_transformation = None
    
    # Load all candidate point clouds
    candidate_files = glob.glob(f"{candidate_pcd_folder}/*.npy")

    for candidate_file in candidate_files:
        print("Processing:", candidate_file)
        candidate_pcd = load_npy_file_as_point_cloud(candidate_file)
        
        transformation, fitness, rmse = icp_align(candidate_pcd, target_pcd)
        score = w_fit * fitness - w_rmse * rmse  # Higher fitness, lower RMSE is better
        # Apply ICP registration
        # icp_result = o3d.pipelines.registration.registration_icp(
        #     candidate_pcd, target_pcd, max_correspondence_distance,
        #     init=np.eye(4),
        #     criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
        # )

        # Check if this candidate is a better match
        # if icp_result.fitness > best_fitness or (icp_result.fitness == best_fitness and icp_result.inlier_rmse < best_rmse):
        #     best_fitness = icp_result.fitness
        #     best_rmse = icp_result.inlier_rmse
        #     best_pcd_file = candidate_file
        #     best_transformation = icp_result.transformation
        
        #visualize the aligned point cloud
        # o3d.visualization.draw_geometries([candidate_pcd.transform(transformation), target_pcd])
        print("RMSE:", rmse)    
        print("Score:", score)
            
        if score > best_score:
            best_score = score
            best_fitness = fitness
            best_rmse = rmse
            best_pcd_file = candidate_file
            best_transformation = transformation

    return best_pcd_file, best_fitness, best_rmse, best_transformation, best_score

# Specify paths
target_pcd_path = "data/scene_data/bottle_coconut_test_scene_data/test_scene_filtered_point_cloud.npy" #"path/to/target.pcd"
candidate_pcd_folder = "data/pointcloud_data/candidiate_objects"

# Find the best matching file
best_pcd_file, best_fitness, best_rmse, best_transformation, best_score = find_best_matching_pcd(target_pcd_path, candidate_pcd_folder)

print("Best Matching PCD File:", best_pcd_file)
print("Best Fitness:", best_fitness)
print("Best Transformation Matrix:\n", best_transformation)
print("Best RMSE:", best_rmse)
print("Best Score:", best_score)
