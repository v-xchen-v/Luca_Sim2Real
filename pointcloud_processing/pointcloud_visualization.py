import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

def visualize_numpy_as_point_cloud(points: np.ndarray) -> None:
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

    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window('Point Cloud Visualization', width=800, height=600)
    vis.add_geometry(point_cloud)

    # Handle the visualization loop manually
    while not vis.poll_events():
        vis.update_renderer()

    # Close the visualization window
    vis.destroy_window()
    
    
def visualize_point_cloud(pcd: o3d.geometry.PointCloud):
    """
    Visualize the given point cloud.

    Args:
    - pcd: The point cloud to visualize as an Open3D PointCloud object.
    """

    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window('Point Cloud Visualization', width=800, height=600)
    vis.add_geometry(pcd)

    # Handle the visualization loop manually
    while not vis.poll_events():
        vis.update_renderer()

    # Close the visualization window
    vis.destroy_window()