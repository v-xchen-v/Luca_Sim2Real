import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

VIS_OBJECT_COORDINATE_FRAME = True
ANIM_HAND_RELATIVE_TO_OBJECT = True

import numpy as np
# traj_sim_data = np.load('/mnt/zl_dev/coke_can_1016/step-0.npy', allow_pickle=True)
traj_sim_data = np.load('data/trajectory_data/sim_trajectory/coka_can_1017/step-0.npy', allow_pickle=True)

from coordinates.transformation_utils import create_transformation_matrix, create_relative_transformation
from scipy.spatial.transform import Rotation as R
from pytransform3d.transformations import invert_transform, concat

# Extract the base link(right hand base) position from the trajectory
## [x, y, z, qx, qy, qz, qw] for translation and quaternion rotation
right_hand_base_sim = traj_sim_data.item()['right_hand_base_pose_buf'] # [num_steps, 1, 7]
right_hand_base_sim = right_hand_base_sim.squeeze(axis=1) # [num_steps, 7]
T_right_hand_base_real_to_sim = np.eye(4)
T_right_hand_base_real_to_sim[:3, :3] = R.from_rotvec([-np.pi/2, 0, 0]).as_matrix()

# # Extract the grasp flag from the trajectory
# grasp_flag = traj_sim_data.item()['hold_flag_buf'] # [num_steps, 1, 1]
# grasp_flag.squeeze(axis=1) # [num_steps, 1]

# Extract the object position from the trajectory
object_pos = traj_sim_data.item()['object_pose_buf'] # [num_steps, 1, 7]
object_pos = object_pos.squeeze(axis=1) # [num_steps, 7]

def get_frame_data(i=0):
    object_pos_0 = object_pos[i, :]
    object_pos_0_wxyz = object_pos_0[3:]
    object_pos_0_xyzw = object_pos_0_wxyz[[1, 2, 3, 0]]
    right_hand_base_pos_0 = right_hand_base_sim[i, :]
    right_hand_base_pos_0_wxyz = right_hand_base_pos_0[3:]
    right_hand_base_pos_0_xyzw = right_hand_base_pos_0_wxyz[[1, 2, 3, 0]]

    right_hand_base_0_frame = create_transformation_matrix(
        right_hand_base_pos_0[:3], 
        R.from_quat(right_hand_base_pos_0_xyzw).as_matrix())
    object_0_frame = create_transformation_matrix(
        object_pos_0[:3], 
        R.from_quat(object_pos_0_xyzw).as_matrix())
    # T_right_hand_base_sim_to_object = create_relative_transformation(object_0_frame, right_hand_base_0_frame)
    
    # Apply the transformation from real to sim
    right_hand_base_0_frame_in_real = concat(invert_transform(T_right_hand_base_real_to_sim), right_hand_base_0_frame)
    return object_0_frame, right_hand_base_0_frame, right_hand_base_0_frame_in_real

if VIS_OBJECT_COORDINATE_FRAME:
    from coordinates.visualization import visualize_frame
    object_frame, _, _ = get_frame_data(0)
    visualize_frame(object_frame, "Object in the world")

if ANIM_HAND_RELATIVE_TO_OBJECT:
    from matplotlib import pyplot as plt
    from pytransform3d.transformations import plot_transform
    from matplotlib.animation import FuncAnimation
    def visualize_frames():
        """
        Plot frames with same world coordinate np.eye(4).
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Set plot limits and labels
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([0, 2])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        
        def plot_frames(frames, ax):
            for name, T in frames.items():
                plot_transform(ax=ax, A2B=T, s=0.2, name=name)

        def update_frame(i):
            ax.cla()  # Clear the current frame
            ax.set_xlim([-2, 2])
            ax.set_ylim([-2, 2])
            ax.set_zlim([0, 2])
            object_frame, right_hand_base_frame, right_hand_base_frame_in_real = get_frame_data(i)
            # plot_frames({"right_hand_base": right_hand_base_frame, "object": object_frame}, ax)
            plot_frames({"object": object_frame, "right_hand_base": right_hand_base_frame_in_real}, ax)
        # Create animation
        anim = FuncAnimation(fig, update_frame, frames=120, interval=100)

        plt.title("Coordinate Frames Visualization")
        plt.show()
        
visualize_frames()