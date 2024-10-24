"""A object is putting on the sim world ground. We want to visualize the object's coordinate frame in the world coordinate frame."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

VIS_OBJECT_COORDINATE_FRAME = True

import numpy as np
from coordinates.transformation_utils import create_transformation_matrix
from scipy.spatial.transform import Rotation as R
# import cv2

traj_sim_data = np.load('data/trajectory_data/sim_trajectory/coka_can_1017/step-0.npy', allow_pickle=True)
# traj_sim_video = cv2.VideoCapture('data/trajectory_data/sim_trajectory/coka_can_1017/rl-video-step-0.mp4')

# Extract the object position from the trajectory
object_pos = traj_sim_data.item()['object_pose_buf'] # [num_steps, 1, 7]
object_pos = object_pos.squeeze(axis=1) # [num_steps, 7]

def get_frame_data(i=0):
    object_pos_0 = object_pos[i, :]
    object_pos_0_wxyz = object_pos_0[3:]
    object_pos_0_xyzw = object_pos_0_wxyz[[1, 2, 3, 0]]

    object_0_frame = create_transformation_matrix(
        object_pos_0[:3], 
        R.from_quat(object_pos_0_xyzw).as_matrix())

    return object_0_frame

if VIS_OBJECT_COORDINATE_FRAME:
    from coordinates.visualization_utils import visualize_frame
    object_frame = get_frame_data(0)
    visualize_frame(object_frame, "Object in the world")
