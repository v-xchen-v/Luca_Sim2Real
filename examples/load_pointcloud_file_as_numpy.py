import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


from pointcloud_processing.pointcloud_io import load_point_cloud

points = load_point_cloud('./data/debug_data/pointcloud_data/camera_captures/test.ply')
print(points.shape)