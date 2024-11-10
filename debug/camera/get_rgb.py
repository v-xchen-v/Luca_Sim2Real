import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


from pointcloud_processing.realsense_capture import realsense_instance

frame = realsense_instance.get_rgb_frame()
print(frame)