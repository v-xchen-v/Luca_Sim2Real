"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})


adaptor._compute_transformations_with_calibration_data()

adaptor.frame_manager.visualize_known_frames()
adaptor.frame_manager.visualize_transforation("calibration_board_real", "camera_real")

# add real world frame for better visualization for debugging
world_real_frame = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

calibration_board_frame = np.array([[0, -1, 0, 0],
                                    [-1, 0, 0, -1],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])
                
from coordinates.transformations import create_relative_transformation
T_real_world_to_calibration_board = create_relative_transformation(world_real_frame, calibration_board_frame)
adaptor.frame_manager.add_transformation("real_world", "calibration_board_real", T_real_world_to_calibration_board)                
adaptor.frame_manager.add_frame("calibration_board_real", np.eye(4))
                
# Assume that calibration is at the origin
adaptor.frame_manager.visualize_transformations_starting_from(from_frame='calibration_board_real')#, ignore_frames=["calibration_board_real"])
# adaptor.frame_manager.visualize_known_frames()
# adaptor.frame_manager.visualize_transforation("calibration_board_real", "camera_real")