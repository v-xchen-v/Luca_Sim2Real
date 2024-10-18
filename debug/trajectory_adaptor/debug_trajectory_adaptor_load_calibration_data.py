"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

adaptor.get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})