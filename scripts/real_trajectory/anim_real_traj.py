import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


import numpy as np
from scipy.spatial.transform import Rotation as R
from coordinates.visualization_utils import _visualize_frames

VIS_HAND_IN_ROBOT_COORDINATE=True
save_path = "data/trajectory_data/real_trajectory/orange_1024/step-0.npy"

def transform_xyzxyzw_to_matrix(xyzxyzw):
    T = np.eye(4)
    r = R.from_quat(xyzxyzw[3:])
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = xyzxyzw[:3]
    return T

traj_real_data = np.load(save_path)
print(traj_real_data.shape)
robot_base_to_hand_base_xyzxyzw = traj_real_data[:, 0:7]
robot_base_to_hand_base_matrices = [transform_xyzxyzw_to_matrix(item) for item in robot_base_to_hand_base_xyzxyzw]

## Visualize the transformation between robot_right_hand_base in robot_base in real world
if VIS_HAND_IN_ROBOT_COORDINATE:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from pytransform3d.transformations import plot_transform
    import matplotlib
    matplotlib.use('TkAgg')  # Or 'Qt5Agg', depending on your setup
    # Set up the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")



    def update_frame(i):
        ax.cla()  # Clear the current frame
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        # add a text
        # print(f'grasp flag: {grasp_flag_sims[i]}')

        transformation = robot_base_to_hand_base_matrices[i]
        _visualize_frames(ax, {
            "robot_base": np.eye(4),
            "right_hand_base": transformation
        })

    # Create animation
    anim = FuncAnimation(fig, update_frame, frames=int(len(robot_base_to_hand_base_matrices)/5), interval=100)

    # Display the animation
    plt.show(block=True)