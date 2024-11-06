"""
Uses ICP to align and match the point cloud to the closest object from a list of candidate PCBs.
"""
import open3d as o3d
import numpy as np
import os

# TODO: put all candidates here, and do test
candidate_pcds = [
    "data/pointcloud_data/candidiate_objects/coke_can.npy",
]

def center_point_cloud(pcd):
    """Centers the point cloud by subtracting the centroid."""
    centroid = np.mean(np.asarray(pcd.points), axis=0)
    pcd.translate(-centroid)  # Move to the origin
    return pcd, centroid

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, 
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100)
    )
    return pcd_down, fpfh

def icp_align_with_multiple_rotations(source, target, threshold=0.01, max_iterations=50):
    best_transformation = None
    best_fitness = 0
    best_rmse = float('inf')

    rotations = np.array([
        [0, 0, 0],
        [np.pi / 2, 0, 0],
        [np.pi, 0, 0],
        [0, np.pi / 2, 0],
        [0, np.pi, 0],
        [0, 0, np.pi / 2],
        [0, 0, np.pi]
    ])
    max_correspondence_distance = 0.1
    for rotation in rotations:
        # Create initial transformation matrix with rotation
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(rotation)
        initial_transformation = np.eye(4)
        initial_transformation[:3, :3] = rotation_matrix

        # Run ICP with this initial guess
        # icp_result = o3d.pipelines.registration.registration_icp(
        #     source, target, max_correspondence_distance,
        #     init=initial_transformation,
        #     criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        #         max_iteration=1000
        #     )
        # )
        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance, initial_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=2000,relative_fitness=1e-4, relative_rmse=1e-4))

        # Check if this result is better than previous ones
        if icp_result.fitness > best_fitness or (icp_result.fitness == best_fitness and icp_result.inlier_rmse < best_rmse):
            best_transformation = icp_result.transformation
            best_fitness = icp_result.fitness
            best_rmse = icp_result.inlier_rmse

    return best_transformation, best_fitness, best_rmse
    
def icp_align(source, target, threshold=0.01, max_iterations=50):
    """Performs ICP alignment on two centered point clouds."""
    
    target_filtered, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    # # Compute a mesh from the source point cloud
    # source_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(source, alpha=0.05)
    # source_mesh.compute_vertex_normals()

    # # Uniformly sample points from the mesh to upsample the source point cloud
    # upsampled_source = source_mesh.sample_points_uniformly(number_of_points=10000)

    # print(f"Original points: {len(source.points)}, Upsampled points: {len(upsampled_source.points)}")


    # debugging visualization for filtered target
    if False:
        o3d.visualization.draw_geometries([source, target_filtered])
    
    # Estimate initial transformation matrix (identity as starting point)
    init_transformation = np.identity(4)
    
    # # Preprocess both point clouds
    # voxel_size = 0.005
    # source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    # target_down, target_fpfh = preprocess_point_cloud(target_filtered, voxel_size)

    # # Perform global registration
    # distance_threshold = 0.1  # Larger threshold for global registration
    # result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    #     source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    #     4,  # Number of RANSAC iterations
    #     [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
    #     o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
    #     o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    # )

    # # Use the global registration result as the initial transformation for ICP
    # init_transformation = result.transformation
    # print("Initial Transformation from Global Registration:")
    # print(init_transformation)

    # Compute normals if necessary (for ICP with point-to-plane)
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_filtered.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Seems point_to_point is better than point_to_plane
    print("Coarse ICP...")
    # TODO: some object rely on good initial transform, multiple init trans here, and choose the best one
    threshold = 0.1  # Larger threshold for coarse alignment
    # reg_p2p_coarse = o3d.pipelines.registration.registration_icp(
    #     source, target_filtered, threshold, init_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(
    #         max_iteration=2000,relative_fitness=1e-4, relative_rmse=1e-4))
    coarse_transformation, _, _ = icp_align_with_multiple_rotations(source, target_filtered, threshold, max_iterations)
    
    # Use the result as the initial guess for fine alignment
    print("Fine ICP...")
    ## TODO: very important to set threshold_fine, 0.01 is fine for orange, but not for noise coke.
    threshold_fine = 0.01  # Smaller threshold for refinement
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target_filtered, threshold_fine, 
        coarse_transformation, # reg_p2p_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=3000,relative_fitness=1e-6, relative_rmse=1e-6))

    # # Apply ICP
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     source, target_filtered, threshold, init_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint()
    # )
    
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     source, source, threshold, init_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations),
    # )

    # print("ICP converged:", reg_p2p.converged)
    print("ICP Fitness:", reg_p2p.fitness)
    # print("ICP Transformation:\n", reg_p2p.transformation)

    return reg_p2p.transformation, reg_p2p.fitness, reg_p2p.inlier_rmse

def restore_point_cloud(pcd, centroid):
    """Restores the point cloud to its original position."""
    pcd.translate(centroid)  # Move back to the original position
    return pcd

def align_source_to_target(source, target, vis_aligned=False, save_aligned_pcd=False, save_aligned_pcd_path=None):
    """Aligns the source point cloud to the target point cloud."""
    # source is modeling fullview point cloud, and target is paritial view in world
    
    # Center both point clouds at the origin
    source_centered, source_centroid = center_point_cloud(source)
    target_centered, target_centroid = center_point_cloud(target)

    # Perform ICP alignment on centered point clouds
    transformation, fitness, rmse = icp_align(source_centered, target_centered)

    # Apply transformation to the original (non-centered) source
    source_centered.transform(transformation)

    # debugging visualization for visual centered source and target
    # visual centered source and target
    if False:
        o3d.visualization.draw_geometries([source_centered, target_centered])
    
    # Restore both point clouds to their original positions
    aligned_source = restore_point_cloud(source_centered, target_centroid)
    restored_target = restore_point_cloud(target_centered, target_centroid)

    # debugging visualization for visual centered source and target
    if vis_aligned:
        o3d.visualization.draw_geometries([aligned_source, restored_target])
    
    if save_aligned_pcd:
        combined_pcd = aligned_source + restored_target
        # if save path is not provided, save to default path
        if save_aligned_pcd_path is None:
            save_aligned_pcd_path = 'combined_aligned_pcd.pcd'
        
        # if save folder not exist, create it
        if not os.path.exists(os.path.dirname(save_aligned_pcd_path)):
            os.makedirs(os.path.dirname(save_aligned_pcd_path))
        o3d.io.write_point_cloud(save_aligned_pcd_path, combined_pcd)
        print(f"Aligned point cloud saved to {save_aligned_pcd_path}") 
               
    source_to_aligned_rotation_matrix = transformation[:3, :3]
    return aligned_source, restored_target, source_to_aligned_rotation_matrix, fitness, rmse

def align_and_restore(source, target):
    # Step 1: Center both point clouds at the origin
    source_centered, source_centroid = center_point_cloud(source)
    target_centered, target_centroid = center_point_cloud(target)

    # Step 2: Perform ICP alignment on centered point clouds
    transformation, fitness, rmse = icp_align(source_centered, target_centered)

    # Step 3: Apply transformation to the original (non-centered) source
    source_centered.transform(transformation)

    # Step 4: Restore both point clouds to their original positions
    aligned_source = restore_point_cloud(source_centered, source_centroid)
    restored_target = restore_point_cloud(target_centered, target_centroid)

    return aligned_source, restored_target


def get_closest_pcd_match(target_pcd, candidate_pcds, 
                                  max_correspondence_distance=0.1, w_fit=1.0, w_rmse=50.0):
    # TODO: the weight of fitness and rmse should be adjusted based on the object
    
    best_matching_pcd = None
    best_matching_index = -1  # Initialize to indicate no match if not found
    best_fitness = 0
    best_score = -float('inf')
    best_rmse = float('inf')
    best_transformation = None
    
    for i, candidate_pcd in enumerate(candidate_pcds):
        _, _, transformation, fitness, rmse = align_source_to_target(candidate_pcd, target_pcd)
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
            best_matching_pcd = candidate_pcd
            best_transformation = transformation
            best_matching_index = i
            
    return best_matching_pcd, best_matching_index, best_fitness, best_rmse, best_transformation, best_score