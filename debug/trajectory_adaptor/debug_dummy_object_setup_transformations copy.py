"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np
from coordinates.transformation_utils import concat

VIS_CALIBRATION_TRANSFORMATIONS = False

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

# Step1: Get calibration data
adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})


# Step2: Compute all the transformations based on the calibration datas
adaptor._compute_transformations_with_calibration_data()
if VIS_CALIBRATION_TRANSFORMATIONS:
    adaptor.frame_manager.add_frame("calibration_board_real", np.eye(4))
    adaptor.frame_manager.compute_all_frames("calibration_board_real")
    adaptor.frame_manager.compute_all_frames("calibration_board_real")
    adaptor.frame_manager.visualize_known_frames()
    adaptor.frame_manager.visualize_transformation("calibration_board_real", "camera_real")

# add real world frame for better visualization for debugging
world_real_frame = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

calibration_board_frame = np.array([[0, -1, 0, 0],
                                    [-1, 0, 0, -0.1],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

# calibration_board_frame = np.array([[1, 0, 0, 0],
#                                     [0, -1, 0, -0.1],
#                                     [0, 0, -1, 0],
#                                     [0, 0, 0, 1]])
  
T_board_to_camera = adaptor.frame_manager.get_transformation("calibration_board_real", "camera_real")
T_camera_to_robot = adaptor.frame_manager.get_transformation("camera_real", "robot_base_real") 

              
from coordinates.transformation_utils import create_relative_transformation
T_real_world_to_calibration_board = create_relative_transformation(world_real_frame, calibration_board_frame)

T1 = concat(T_board_to_camera, calibration_board_frame)
T2 = concat(T_board_to_camera, T_real_world_to_calibration_board)
print(T1)
# right of manually calcuated cam2bworld
assert np.allclose(T1, T2,  atol=1e-7)

adaptor.frame_manager.add_transformation("real_world", "calibration_board_real", T_real_world_to_calibration_board)                
adaptor.frame_manager.add_frame("real_world", np.eye(4))
T1_by_manger = adaptor.frame_manager.get_transformation("real_world", "camera_real") # also right here.
print(T1_by_manger)
np.allclose(T1, T1_by_manger,  atol=1e-7)
adaptor.frame_manager.visualize_transformation("real_world", "camera_real")
# adaptor.frame_manager.compute_all_frames("real_world")
print(adaptor.frame_manager.frames['camera_real'])
adaptor.frame_manager.print_frames()
# adaptor.frame_manager.visualize_known_frames()
                
# # Assume that calibration is at the origin
# adaptor.frame_manager.compute_all_frames("real_world")
adaptor.frame_manager.visualize_transformations_starting_from(from_frame='real_world')#, ignore_frames=["calibration_board_real"])
# # adaptor.frame_manager.visualize_known_frames()
# adaptor.frame_manager.visualize_transforation("calibration_board_real", "real_world")