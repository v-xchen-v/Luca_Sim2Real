"""A object is putting on the sim world ground. We want to visualize the object's coordinate frame in the world coordinate frame."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

VIS_OBJECT_COORDINATE_FRAME = True

import numpy as np
from coordinates.transformation_utils import create_transformation_matrix
from scipy.spatial.transform import Rotation as R
from pytransform3d.transformations import invert_transform
# import cv2

traj_sim_data = np.load('data/trajectory_data/sim_trajectory/bleach_cleanser_1030/step-0.npy', allow_pickle=True)
# traj_sim_data = np.load('data/trajectory_data/sim_trajectory/realsense_box_1024/step-0.npy', allow_pickle=True)
# traj_sim_video = cv2.VideoCapture('data/trajectory_data/sim_trajectory/coka_can_1017/rl-video-step-0.mp4')

# Extract the object position from the trajectory
object_pos = traj_sim_data.item()['object_pose_buf'] # [num_steps, 1, 7]
object_pos = object_pos.squeeze(axis=1) # [num_steps, 7]
hand_pos = traj_sim_data.item()['right_hand_base_pose_buf'] # [num_steps, 1, 7]
hand_pos = hand_pos.squeeze(axis=1) # [num_steps, 7]

def get_frame_data(pos, i=0):
    pos_0 = pos[i, :]
    pos_0_wxyz = pos_0[3:]
    pos_0_xyzw = pos_0_wxyz[[1, 2, 3, 0]]

    frame0 = create_transformation_matrix(
        pos_0[:3], 
        R.from_quat(pos_0_xyzw).as_matrix())

    return frame0

if VIS_OBJECT_COORDINATE_FRAME:
    from coordinates.visualization_utils import visualize_frames
    object_in_world_frame = get_frame_data(object_pos, 0)
    hand_in_world_frame = get_frame_data(hand_pos, 0) 
    visualize_frames([np.eye(4), object_in_world_frame, hand_in_world_frame], 
                     ["sim_world", "Object in the world", "hand_in_world"])
