"""Verify the logic of using frame manager to massive compute the transofrmations.
"""
import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

import numpy as np
from coordinates.frame_manager import FrameManager
from coordinates.transformations import create_relative_transformation
    
frame_manager = FrameManager()
frame_manager.initialize_frames(["world", "board", "camera"])
world_real_frame = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

calibration_board_frame = np.array([[0, -1, 0, 0],
                                    [-1, 0, 0, -0.1],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])


camera_frame = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 1],
                        [0, 0, 0, 1]])
T_world_to_board = create_relative_transformation(world_real_frame, calibration_board_frame)
T_board_to_camera = create_relative_transformation(calibration_board_frame, camera_frame)

camera_frame_computed = T_board_to_camera @ T_world_to_board
print(f'camera frame computed: {camera_frame_computed}')
assert np.allclose(camera_frame, camera_frame_computed)

frame_manager.add_transformation("world", "board", create_relative_transformation(world_real_frame, calibration_board_frame))
frame_manager.add_transformation("board", "camera", create_relative_transformation(calibration_board_frame, camera_frame))
# would be wrong, carefully distiguish between relative and absolute transformation
# frame_manager.add_transformation("board", "camera", camera_frame)

frame_manager.add_frame("world", np.eye(4))
frame_manager.compute_all_frames("world")
frame_manager.print_frames()
frame_manager.visualize_known_frames()