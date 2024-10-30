"""
Uses ICP to align and match the point cloud to the closest object from a list of candidate PCBs.
"""
import open3d as o3d
import numpy as np

# TODO: put all candidates here, and do test
candidate_pcbs = [
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

def icp_align(source, target, threshold=0.01, max_iterations=1000):
    """Performs ICP alignment on two centered point clouds."""
    
    target_filtered, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
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

    # Apply ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target_filtered, threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     source, source, threshold, init_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations),
    # )

    # print("ICP converged:", reg_p2p.converged)
    print("ICP Fitness:", reg_p2p.fitness)
    print("ICP Transformation:\n", reg_p2p.transformation)

    return reg_p2p.transformation

def restore_point_cloud(pcd, centroid):
    """Restores the point cloud to its original position."""
    pcd.translate(centroid)  # Move back to the original position
    return pcd

def align_source_to_target(source, target):
    """Aligns the source point cloud to the target point cloud."""
    # source is modeling fullview point cloud, and target is paritial view in world
    
    # Center both point clouds at the origin
    source_centered, source_centroid = center_point_cloud(source)
    target_centered, target_centroid = center_point_cloud(target)

    # Perform ICP alignment on centered point clouds
    transformation = icp_align(source_centered, target_centered)

    # Apply transformation to the original (non-centered) source
    source_centered.transform(transformation)

    # visual centered source and target
    o3d.visualization.draw_geometries([source_centered, target_centered])
    
    # Restore both point clouds to their original positions
    aligned_source = restore_point_cloud(source_centered, target_centroid)
    restored_target = restore_point_cloud(target_centered, target_centroid)

    return aligned_source, restored_target

def align_and_restore(source, target):
    # Step 1: Center both point clouds at the origin
    source_centered, source_centroid = center_point_cloud(source)
    target_centered, target_centroid = center_point_cloud(target)

    # Step 2: Perform ICP alignment on centered point clouds
    transformation = icp_align(source_centered, target_centered)

    # Step 3: Apply transformation to the original (non-centered) source
    source_centered.transform(transformation)

    # Step 4: Restore both point clouds to their original positions
    aligned_source = restore_point_cloud(source_centered, source_centroid)
    restored_target = restore_point_cloud(target_centered, target_centroid)

    return aligned_source, restored_target

