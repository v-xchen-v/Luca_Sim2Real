import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud
import open3d as o3d

object_pcd = load_npy_file_as_point_cloud("data/pointcloud_data/candidiate_objects/sunscreen.npy")
# draw axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# object_pcd += axis
o3d.visualization.draw_geometries([object_pcd, axis])