import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from calibration.visualize_table_to_camera import visualize_table_to_camera
from calibration.calibration_data_loader import load_table_to_camera_extrinsics_from_npy

# Example usage
extrinsics_filedir = "data/debug_data/calibration/output/checkerboard_002"

# Load the extrinsics from YAML
T_table_to_camera = load_table_to_camera_extrinsics_from_npy(extrinsics_filedir)

# Visualize the transformation
visualize_table_to_camera(T_table_to_camera)
