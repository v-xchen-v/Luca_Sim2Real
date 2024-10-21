"""Calibration data is ready, now compute all the transformations based on the calibration data, includes:
- table to camera
- camera to robot
"""
import numpy as np

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera2", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})

adaptor.frame_manager.print_frames()
adaptor.frame_manager.print_transformations()

adaptor._compute_transformations_with_calibration_data()


adaptor.frame_manager.print_frames()
adaptor.frame_manager.print_transformations()
# adaptor.frame_manager.visualize_known_frames()

# Assume that calibration is at the origin
adaptor.frame_manager.add_frame("calibration_board_real", np.eye(4))
adaptor.frame_manager.visualize_transformations(
    [
        ('calibration_board_real', 'camera_real'),
        ('camera_real', 'robot_base_real')], block=True)