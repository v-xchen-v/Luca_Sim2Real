import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


import open3d as o3d
from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud

pcd_files = [
    "data/scene_data/orange_test_scene_data_1031_001_fail/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_002_success/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_003_success_after_fix_init/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_004_success/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_005_success/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_006_fail/test_scene_filtered_point_cloud.npy",
    "data/scene_data/orange_test_scene_data_1031_007_fail/test_scene_filtered_point_cloud.npy",
    # Add more PCD files as needed
]

# Load multiple PCDs
pcd_list = []
for i, pcd_file in enumerate(pcd_files):
    pcd = load_npy_file_as_point_cloud(pcd_file)
    pcd_list.append(pcd)

# Apply transformations if needed (for example, to avoid overlap)
# For example: translate pcd2 along the x-axis
pcd_fail_list = [
    pcd_list[0], 
    pcd_list[5],
    pcd_list[6]
]
for i, pcd in enumerate(pcd_list):
    pcd.translate((i, 0, 0))  # Adjust the translation as needed

for i, pcd in enumerate(pcd_fail_list):
    pcd.translate((0, 1, 0))  # Adjust the translation as needed
    
# Visualize all point clouds together
o3d.visualization.draw_geometries(pcd_list)
