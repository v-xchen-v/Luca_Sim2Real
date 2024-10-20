"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np
from coordinates.transformations import concat

VIS_CALIBRATION_TRANSFORMATIONS = False

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

# Step1: Get calibration data
adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})


# Step2: Compute all the transformations based on the calibration datas
adaptor.add_transfromations_with_calibration()
# if VIS_CALIBRATION_TRANSFORMATIONS:
#     adaptor.frame_manager.add_frame("calibration_board_real", np.eye(4))
#     adaptor.frame_manager.compute_all_frames("calibration_board_real")
#     adaptor.frame_manager.compute_all_frames("calibration_board_real")
#     adaptor.frame_manager.visualize_known_frames()
#     adaptor.frame_manager.visualize_transforation("calibration_board_real", "camera_real")


# Optional Step: Add a readable frame at same position but different orientation with calibration_board_real instead align with world coordinate axes for better visualization for debugging.
readable_real_frame = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

calibration_board_frame = np.array([[0, -1, 0, 0],
                                    [-1, 0, 0, -0.1],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

from coordinates.transformations import create_relative_transformation
T_readable_real_to_calibration_board = create_relative_transformation(readable_real_frame, calibration_board_frame)

adaptor.frame_manager.add_transformation("readable_real", "calibration_board_real", T_readable_real_to_calibration_board)                
adaptor.frame_manager.add_frame("readable_real", np.eye(4))
# adaptor.frame_manager.visualize_known_frames()
T1_by_manger = adaptor.frame_manager.get_transformation("readable_real", "camera_real") # also right here.
print(T1_by_manger)
# adaptor.frame_manager.visualize_transforation("real_world", "camera_real")
adaptor.frame_manager.visualize_transformations(
    [('readable_real', 'calibration_board_real'),
     ('calibration_board_real', 'camera_real'),
     ('camera_real', 'robot_base_real')])
