"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np
from coordinates.transformation_utils import concat

VIS_CALIBRATION_TRANSFORMATIONS = False
VIS_READABLE_FRAME_SETUP = False
VIS_OBJECT_IN_REAL = False
VIS_HAND_OBJECT_RELATIVE = True
VIS_ANIM_HAND_APPROACH_OBJECT = True

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

# Step1: Get calibration data
adaptor._get_calibration_data(calibration_data_dir="calibration/calibration_data/camera1", overwrite_if_exists=False, calibration_board_info={
    "pattern_size": (5, 8),
    "square_size": 0.03
})


# Step2: Compute all the transformations based on the calibration datas
adaptor.add_transfromations_with_calibration()
if VIS_CALIBRATION_TRANSFORMATIONS:
    adaptor.frame_manager.visualize_transformations([("calibration_board_real", "camera_real"),
                                                     ("camera_real", "robot_base_real")])


# Optional Step: Add a readable frame at same position but different orientation with calibration_board_real instead align with world coordinate axes for better visualization for debugging.
readable_real_frame = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
## For translation respect, it means that the calibration_board_real frame is 0.03*4 meters away from the readable_real_frame in the x direction.
calibration_board_frame = np.array([[0, -1, 0, 0.03*4],
                                    [-1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

from coordinates.transformation_utils import create_relative_transformation, create_transformation_matrix
from scipy.spatial.transform import Rotation as R
T_readable_real_to_calibration_board = create_relative_transformation(readable_real_frame, calibration_board_frame)

adaptor.frame_manager.add_transformation("readable_real", "calibration_board_real", T_readable_real_to_calibration_board)    
if VIS_READABLE_FRAME_SETUP:            
    adaptor.frame_manager.visualize_transformations(
        [('readable_real', 'calibration_board_real'),
        ('calibration_board_real', 'camera_real'),
        ('camera_real', 'robot_base_real')])

# Step 3: Object setup, Assume we put the object at the origin of readable_real frame   
object_rot_vec = [0, 0, -np.pi/2]
T_object_to_readable = create_transformation_matrix([0, 0, 0], R.from_rotvec(object_rot_vec).as_matrix())
adaptor.frame_manager.add_transformation("object_real", "readable_real", T_object_to_readable)
if VIS_OBJECT_IN_REAL:
    adaptor.frame_manager.visualize_transformations([
        ('readable_real', 'calibration_board_real'),
        ('calibration_board_real', 'object_real'),
    ])

# Step 4: Load the simulated trajectory and compute the object relative to the right hand base at first step
T_right_hand_base_sim_to_object, _, right_hand_base_pos_sim, _ = adaptor.parse_sim_trajectory(r'data/trajectory_data/sim_trajectory/coka_can_1017/step-0.npy')
T_right_hand_base_real_to_robot_base = adaptor.compute_constrained_object_relative_to_right_hand_base(T_right_hand_base_sim_to_object)
adaptor.frame_manager.add_transformation("right_hand_base_step0_real", "robot_base_real", T_right_hand_base_real_to_robot_base)
if VIS_HAND_OBJECT_RELATIVE:
    adaptor.frame_manager.visualize_transformations([
            # ('readable_real', 'calibration_board_real'),
            # ('calibration_board_real', 'camera_real'),
            # ('camera_real', 'robot_base_real'),
            # ('robot_base_real', 'right_hand_base_real'),
            ('readable_real', 'right_hand_base_step0_real'),
            # ('right_hand_base_real', 'object_real')
        ])

# Step 5: Compute transformation between robot_right_hand_base to robot_base in real world
T_robot_right_hand_real_to_robot_steps = adaptor.compute_right_hand_base_to_object(right_hand_base_pos_sim)   
if VIS_ANIM_HAND_APPROACH_OBJECT:
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
        transformation = T_robot_right_hand_real_to_robot_steps[i]
        # plot_transform(ax, A2B=transformation, s=0.1)  # s sets the size of the frame
        adaptor.frame_manager.update_transformation("right_hand_base_real", "object_real", transformation)
        adaptor.frame_manager.visualize_transformations([
            ("readable_real", "right_hand_base_real")], ax)

    # Create animation
    anim = FuncAnimation(fig, update_frame, frames=len(T_robot_right_hand_real_to_robot_steps), interval=100)

    # Display the animation
    plt.show(block=True)

# Step 5: Compute the executable trajectory in real world
# adaptor.map_sim_to_real_trajectory()


