import open3d as o3d

# Load and visualize the saved point cloud
pcd = o3d.io.read_point_cloud("data/scene_data/test_scene_data/test_scene.pcd")
o3d.visualization.draw_geometries([pcd])