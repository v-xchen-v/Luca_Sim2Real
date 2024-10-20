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

adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})

adaptor.frame_manager.print_frames()
adaptor.frame_manager.print_transformations()

adaptor._compute_transformations_with_calibration_data()


adaptor.frame_manager.print_frames()
adaptor.frame_manager.print_transformations()
adaptor.frame_manager.visualize_known_frames()

# Assume that calibration is at the origin
# adaptor.frame_manager.add_frame("calibration_board_real", np.eye(4))
# adaptor.frame_manager.visualize_transformations_starting_from(from_frame='calibration_board_real')

adaptor._compute_transformations_with_dummy_object_setup([0, 0, 0], None)
adaptor.frame_manager.print_transformations()
# adaptor.frame_manager.add_frame("object_real", np.eye(4))
# adaptor.frame_manager.visualize_transformations_starting_from(from_frame="object_real")


T_right_hand_base_sim_to_object, _, _, _ = adaptor.parse_sim_trajectory(r'data/trajectory_data/sim_trajectory/coka_can_1017/step-0.npy')
adaptor.compute_constrained_object_relative_to_right_hand_base(T_right_hand_base_sim_to_object)
adaptor.frame_manager.add_frame("right_hand_base_real", np.eye(4))
adaptor.frame_manager.visualize_transformations_starting_from(from_frame="right_hand_base_real")