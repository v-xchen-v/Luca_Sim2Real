import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

    
from pointcloud_processing.pointcloud_io import save_point_cloud_from_realsense

save_point_cloud_from_realsense('./data/debug_data/pointcloud_data/camera_captures', 'test', overwrite_if_exists=True)