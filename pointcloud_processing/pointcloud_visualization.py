import open3d as o3d
import numpy as np

def visualize_point_cloud(points: np.ndarray) -> None:
    """
    Visualize the given point cloud.

    Args:
    - points: The point cloud to visualize as a NumPy array.
    """
    # check if the points is empty
    if points.size == 0:
        raise ValueError("The input point cloud is empty.")
    
    if points.shape[1] == 4:
        # remove the homogeneous coordinate, [x, y, z, 1] -> [x, y, z]
        points = points[:, :3]
        
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([point_cloud])