# Transformation Matrices between frames

import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# Helper function to create a 4x4 transformation matrix
def create_transformation_matrix(translation, rotation_matrix=None):
    """
    Create a 4x4 homogeneous transformation matrix from translation and rotation.

    Parameters:
    - translation: A list or array of 3 elements [x, y, z] representing the translation vector.
    - rotation_matrix: A 3x3 rotation matrix (optional). If not provided, the function will 
                       use an identity rotation matrix, meaning no rotation.

    Returns:
    - T: A 4x4 homogeneous transformation matrix that combines the given rotation and translation.
    """
    T = np.eye(4)  # Initialize a 4x4 identity matrix as the base transformation.

    if rotation_matrix is not None:
        # Insert the 3x3 rotation matrix into the top-left corner of the 4x4 matrix.
        T[:3, :3] = rotation_matrix

    # Insert the 3-element translation vector into the last column (except the bottom row).
    T[:3, 3] = translation

    # Return the resulting 4x4 transformation matrix.
    return T

# Helper function to create a 4x4 transformation matrix from source frame to target frame
def relative_transformation(source_matrix, target_matrix):
    """
    Create a 4x4 homogeneous transformation matrix from a source frame to a target frame.

    Parameters:
    - source_frame: A 4x4 homogeneous transformation matrix representing the source frame.
    - target_frame: A 4x4 homogeneous transformation matrix representing the target frame.

    Returns:
    - T: A 4x4 homogeneous transformation matrix that transforms points from the source frame to the target frame.
    """
    # Invert the source matrix to get the transformation from source to world
    source_inv = np.linalg.inv(source_matrix)
    
    # Compute the relative transformation from source to target
    T_source_to_target = np.dot(source_inv, target_matrix)

    # Return the resulting 4x4 transformation matrix.
    return T_source_to_target

# Example: Rotation around the Z-axis by 45 degrees
theta = np.radians(45)
rotation_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Define all the transformations in the table
## Transformation between simulation and real world: Make them the same
T_sim_to_real = create_transformation_matrix([0, 0, 0], None)
T_real_to_sim = np.linalg.inv(T_sim_to_real)  # Inverse of sim to real

# Transformation from calibration
## table calibration
T_calibration_board_real_to_camera = np.load("data/debug_data/calibration/output/checkerboard_002/table_to_camera.npy")


## Compute the transformation from table_real to camera_real
# T_table_real_to_camera_real = 
T_robot_base_real_to_world = create_transformation_matrix([0, 0, 1])
T_robot_base_sim_to_sim_world = create_transformation_matrix([0.5, 0.5, 0])

T_robot_base_real_to_sim_world = create_transformation_matrix([1, 0, 1], rotation_z)

T_robot_right_hand_real_to_robot = create_transformation_matrix([0.2, 0, 0.3])
T_robot_right_hand_sim_to_robot = create_transformation_matrix([0.3, 0.1, 0])

T_robot_right_hand_real_to_world = create_transformation_matrix([1.5, 0.5, 1.3])
T_robot_right_hand_sim_to_sim_world = create_transformation_matrix([0.6, 0.4, 0])

T_object_real_to_world = create_transformation_matrix([2, 1, 0])
T_object_sim_to_sim_world = create_transformation_matrix([1, 1, 1])

T_object_real_to_robot = create_transformation_matrix([0.1, 0.2, 0.3])
T_object_sim_to_robot = create_transformation_matrix([0.4, 0.3, 0.2])

T_camera_real_to_world = create_transformation_matrix([3, 2, 1])
T_camera_real_to_robot = create_transformation_matrix([0.5, 0.5, 1])

T_camera_real_to_object = create_transformation_matrix([0.2, 0.1, 0])

T_object_sim_to_robot_right_hand_base = create_transformation_matrix([0.4, 0.4, 0.1])
T_object_real_to_robot_right_hand_base = create_transformation_matrix([1, 1, 0.2])

# # Visualization of all the transformations
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# List of transformations to plot
transforms = {
    "T_sim_to_real": T_sim_to_real,
    # "sim_world": np.eye(4),
    # "real_world": T_sim_to_real,
    # "robot_base_real": T_robot_base_real_to_world,
    # "robot_base_sim": T_robot_base_sim_to_sim_world,
    # "right_hand_real": T_robot_right_hand_real_to_world,
    # "right_hand_sim": T_robot_right_hand_sim_to_sim_world,
    # "object_real": T_object_real_to_world,
    # "object_sim": T_object_sim_to_sim_world,
    # "camera_real": T_camera_real_to_world,
}

# # Plot all frames
# for name, T in transforms.items():
#     plot_transform(ax=ax, A2B=T, name=name, s=0.3)

# # Customize plot limits and labels
# ax.set_xlim([-2, 4])
# ax.set_ylim([-2, 4])
# ax.set_zlim([-1, 3])

# ax.set_xlabel("X-axis")
# ax.set_ylabel("Y-axis")
# ax.set_zlabel("Z-axis")
# plt.title("Transformations Between Frames")

# # Show the plot
# plt.show()
