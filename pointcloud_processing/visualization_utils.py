import numpy as np
import open3d as o3d

def show_points_in_window(points, colors=None):
    """
    Visualizes the points in a window.

    Args:
        points (np.ndarray): The points to visualize.
        colors (np.ndarray): The colors of the points.
    """
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])