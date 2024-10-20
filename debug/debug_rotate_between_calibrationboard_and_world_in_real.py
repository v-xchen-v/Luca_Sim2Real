import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

import numpy as np
from scipy.spatial.transform import Rotation as R
from coordinates.transformation_utils import create_transformation_matrix

# rotation_matrix = R.from_rotvec([np.pi, 0, -np.pi]).as_matrix()
# print(rotation_matrix)

rotation_matrix = np.array([[0, -1, 0],
                            [-1, 0, 0],
                            [0, 0, -1]])
print(R.from_matrix(rotation_matrix).as_rotvec()*180/np.pi)

from coordinates.frame_manager import FrameManager

frame_manager = FrameManager()

"""The rotation vector r=[127.28,−127.28,0.0]r=[127.28,−127.28,0.0] represents:

    A 180-degree rotation.
    About the axis: [0.707,−0.707,0.0][0.707,−0.707,0.0], which lies along the diagonal in the xy-plane, pointing between the x- and negative y-axes.
"""
frame_manager.initialize_frames(["calibration_board", "world"])
frame_manager.add_transformation("world", "calibration_board", 
                                 create_transformation_matrix([0, -1, 0], R.from_rotvec(R.from_matrix(rotation_matrix).as_rotvec()).as_matrix()))
frame_manager.add_frame("world", np.eye(4))
frame_manager.visualize_transformations_starting_from("world")