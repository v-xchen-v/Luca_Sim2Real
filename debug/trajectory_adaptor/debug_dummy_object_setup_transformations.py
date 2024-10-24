"""Precomputed camera intrinsic and camera to robot transformation, and capture a frame to compute table-to-camera transformation."""

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np
from coordinates.transformation_utils import concat
from pytransform3d.transformations import invert_transform

VIS_CALIBRATION_TRANSFORMATIONS = True
VIS_READABLE_FRAME_SETUP = True
VIS_OBJECT_IN_REAL = True
VIS_HAND_OBJECT_RELATIVE = True
VIS_ANIM_HAND_APPROACH_OBJECT = True
VIS_HAND_XYZ_IN_ROBOT_COORDINATE = True
VIS_HAND_IN_ROBOT_COORDINATE = True

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

CAMERA = "camera2"
CAPTURE_NEW_TABLE_CALIBRATION = False
CALIBRATION_BOARD_PATTERN_SIZE = (8, 11)
CALIBRATION_BOARD_SQUARE_SIZE = 0.02
# CALIBRATION_BOARD_PATTERN_SIZE = (5, 8)
# CALIBRATION_BOARD_SQUARE_SIZE = 0.03

# Step1: Get calibration data
adaptor._get_calibration_data(calibration_data_dir=f"calibration/calibration_data/{CAMERA}", overwrite_if_exists=CAPTURE_NEW_TABLE_CALIBRATION, calibration_board_info={
    # "pattern_size": (5, 8),
    # "square_size": 0.03
    "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
    "square_size": CALIBRATION_BOARD_SQUARE_SIZE,
}, error_threshold=1)


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
calibration_board_frame = np.array([[0, -1, 0, 0],
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
# object_rot_vec = [0, 0, -np.pi/2]
object_rot_vec = [0, 0, -np.pi/2]
T_object_to_readable = create_transformation_matrix([0, 0, -0.05], R.from_rotvec(object_rot_vec).as_matrix())
adaptor.frame_manager.add_transformation("object_real", "readable_real", T_object_to_readable)
if VIS_OBJECT_IN_REAL:
    adaptor.frame_manager.visualize_transformations([
        ('readable_real', 'calibration_board_real'),
        ('calibration_board_real', 'object_real'),
    ])
# Right until here
#---------------------- Real world setup done----------------------#

#---------------------- Simulated world setup ----------------------#
# Step 4: Build up 'sim_world', 'object_sim', 'right_hand_base_sim' frames #with respect to readable_real frame
traj_name = "coka_can_1017"
driven_hand_pos_sim, right_hand_base_pos_sim, grasp_flag_sims = adaptor.parse_sim_trajectory(f'data/trajectory_data/sim_trajectory/{traj_name}/step-0.npy')

# Visualize to make sure that the 'world', 'object', 'hand_base' in sim is built correctly same as in Isaac Gym
# Next is to built the actions in Isaac Gym 
adaptor.frame_manager.visualize_transformations(
    [
        ("sim_world", "object_sim"),
        ("object_sim", "right_hand_base_sim"),
    ]
)
# Step 5: Build up 'object_sim', 'right_hand_base_sim' relative pos across steps
T_right_hand_base_to_object_steps_in_sim = adaptor.compute_right_hand_base_to_object_steps_in_sim(right_hand_base_pos_sim)
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
    T_object_in_world = adaptor.frame_manager.get_transformation("sim_world", "object_sim") # or aka, T_object_world_to_object
    transformation = invert_transform(T_right_hand_base_to_object_steps_in_sim[i])
    
    from coordinates.visualization_utils import _visualize_frames
    _visualize_frames(ax, {
        "sim_world": np.eye(4),
        "object_sim": T_object_in_world,
        "right_hand_base_sim": transformation
    }, limits=[[-0.5, 0.5],
               [-0.5, 0.5],
               [-0.5, 0.5]])
    
    # plot_transform(A2B=transformation, ax=ax, s=0.1)  # s sets the size of the frame
    # # plot_transform(ax, A2B=transformation, s=0.1)  # s sets the size of the frame
    # adaptor.frame_manager.update_transformation("right_hand_base_real", "object_real", transformation)
    # # show the dynamic transformation of the object, right_hand_base_real, with respect to readable_real frame 
    # # and optional reference robot_base_real frame and camera_real frame
    # adaptor.frame_manager.visualize_transformations(
    #     [
    #         ("readable_real", 'object_real'),
    #         ('object_real',"right_hand_base_real"),
    #         ("right_hand_base_real", "robot_base_real"),
    #         ("robot_base_real", "camera_real"),
    #     ], 
    #     ax, block=False)

# Create animation
anim = FuncAnimation(fig, update_frame, frames=int(len(T_right_hand_base_to_object_steps_in_sim)/5), interval=100)

# Display the animation
plt.show(block=True)

# Step 5: Compute transformation between robot_right_hand_base to robot_base in real world
T_robot_right_hand_real_to_robot_steps, T_robot_base_to_right_hand_base_steps_sim = adaptor.compute_right_hand_base_to_object(right_hand_base_pos_sim)   
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
        # show the dynamic transformation of the object, right_hand_base_real, with respect to readable_real frame 
        # and optional reference robot_base_real frame and camera_real frame
        adaptor.frame_manager.visualize_transformations(
            [
                ("readable_real", 'object_real'),
                ('object_real',"right_hand_base_real"),
                ("right_hand_base_real", "robot_base_real"),
                ("robot_base_real", "camera_real"),
            ], 
            ax, block=False)

    # Create animation
    anim = FuncAnimation(fig, update_frame, frames=int(len(T_robot_right_hand_real_to_robot_steps)/5), interval=100)

    # Display the animation
    plt.show(block=True)

# Step 5: Load the simulated trajectory and compute the object relative to the right hand base at first step
T_right_hand_base_real_to_robot_base = adaptor.compute_constrained_object_relative_to_right_hand_base()
adaptor.frame_manager.add_transformation("right_hand_base_step0_real", "robot_base_real", T_right_hand_base_real_to_robot_base)
if VIS_HAND_OBJECT_RELATIVE:
    adaptor.frame_manager.visualize_transformations([
            # ('readable_real', 'calibration_board_real'),
            # ('calibration_board_real', 'camera_real'),
            # ('camera_real', 'robot_base_real'),
            # ('robot_base_real', 'right_hand_base_real'),
            ('readable_real', 'right_hand_base_step0_real'),
            ('right_hand_base_step0_real', 'object_real')
        ])



# Step 5: Save the executable trajectory in real world
## Save the transformation between robot_right_hand_base to robot_base in real world and joint angles of hand

### Convert 4x4 to [x, y, z, roll, pitch, yaw]
def transform_to_xyzrpy(transform):
    xyz = transform[:3, 3]
    rpy = R.from_matrix(transform[:3, :3]).as_quat()
    return np.concatenate([xyz, rpy])



# T_right_hand_base_real_to_sim = np.eye(4)
# T_right_hand_base_real_to_sim[:3, :3] = R.from_rotvec([np.pi/2, 0, 0]).as_matrix()
# T_right_hand_base_sim_to_real = invert_transform(T_right_hand_base_real_to_sim)
# T_robot_base_to_right_hand_base_steps_real = [concat(T, T_right_hand_base_sim_to_real) for T in T_robot_base_to_right_hand_base_steps_sim]
# T_robot_base_to_right_hand_base_steps_real_xyzrpy = [transform_to_xyzrpy(T) for T in T_robot_base_to_right_hand_base_steps_real]

T_robot_base_to_right_hand_base_steps_sim_xyzrpy = [transform_to_xyzrpy(T) for T in T_robot_base_to_right_hand_base_steps_sim]


### Create a npy dict
# traj_real_dict = {
#     "T_right_hand_base_real_to_robot_steps": T_robot_right_hand_real_to_robot_steps, # [num_steps, 7]
#     "driven_hand_pos_sim": driven_hand_pos_sim, # [num_steps, 6]
#     "grasp_flag_sims": grasp_flag_sims # [num_steps, 1]
# }

# concat T_robot_right_hand_real_to_robot_step, driven_hand_pos_sim, and grasp grasp_flag_sims

# Check T_right_hand_base_steps_sim_to_robot_base's x, y, z
if VIS_HAND_XYZ_IN_ROBOT_COORDINATE:
## plot the x, y, z of T_right_hand_base_steps_sim_to_robot_base
    import matplotlib.pyplot as plt
    T_robot_base_to_right_hand_base_steps_sim_xyzrpy = np.array(T_robot_base_to_right_hand_base_steps_sim_xyzrpy)
    plt.plot(T_robot_base_to_right_hand_base_steps_sim_xyzrpy[:, 0], label="x")
    plt.plot(T_robot_base_to_right_hand_base_steps_sim_xyzrpy[:, 1], label="y")
    plt.plot(T_robot_base_to_right_hand_base_steps_sim_xyzrpy[:, 2], label="z")
    plt.legend()
    plt.show()

traj_real_data = np.concatenate([T_robot_base_to_right_hand_base_steps_sim_xyzrpy, driven_hand_pos_sim, grasp_flag_sims], axis=1)

# traj_real_data = np.concatenate([T_robot_base_to_right_hand_base_steps_real_xyzrpy, driven_hand_pos_sim, grasp_flag_sims], axis=1)

save_path = f"data/trajectory_data/real_trajectory/{traj_name}/step-0.npy"
if not os.path.exists(os.path.dirname(save_path)):
    os.makedirs(os.path.dirname(save_path))
np.save(save_path, traj_real_data)

# Step 6: Reload and check the shape and content of data
traj_real_data = np.load(save_path)
print(traj_real_data.shape)

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
        print(f'grasp flag: {grasp_flag_sims[i]}')
        
        transformation = T_robot_base_to_right_hand_base_steps_sim[i]
        # plot_transform(ax, A2B=transformation, s=0.1)  # s sets the size of the frame
        adaptor.frame_manager.update_transformation("robot_base_real", "right_hand_base_real", transformation)
        # show the dynamic transformation of the object, right_hand_base_real, with respect to readable_real frame 
        # and optional reference robot_base_real frame and camera_real frame
        adaptor.frame_manager.visualize_transformations(
            [
                ("robot_base_real", "right_hand_base_real"),
                ("right_hand_base_real", "object_real"),
                ("object_real", "readable_real"),
            ], 
            ax, block=False)

    # Create animation
    anim = FuncAnimation(fig, update_frame, frames=int(len(T_robot_base_to_right_hand_base_steps_sim)/5), interval=100)

    # Display the animation
    plt.show(block=True)