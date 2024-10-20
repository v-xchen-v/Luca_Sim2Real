"""As the example A, B, C frame, use the @ is wrong but concat is corrent by 
the rotation orthogonality requirement?"""

import numpy as np
from pytransform3d.transformations import (invert_transform, concat)
from scipy.spatial.transform import Rotation as R

def orthogonalize_rotation(R_matrix):
    """Ensure the rotation matrix is orthogonal."""
    # Use scipy to find the closest valid rotation matrix
    rotation = R.from_matrix(R_matrix)
    return rotation.as_matrix()

def create_relative_transformation(source_matrix, target_matrix):
    """Create a 4x4 transformation matrix from source frame to target frame."""
    source_inv = np.linalg.inv(source_matrix)
    return  np.dot(source_inv, target_matrix)

def create_relative_transformation_orth_rotation(source_matrix, target_matrix):
    """Create a 4x4 transformation matrix from source frame to target frame."""
    source_inv = np.linalg.inv(source_matrix)
    
        # Perform matrix multiplication
    T1=source_inv
    T2=target_matrix
    T12 = np.dot(T1, T2)

    # Extract the rotation and translation parts
    R_matrix = T12[:3, :3]
    t_vector = T12[:3, 3]

    # Orthogonalize the rotation part to avoid drift
    R_matrix = orthogonalize_rotation(R_matrix)

    # Rebuild the 4x4 transformation matrix
    T12[:3, :3] = R_matrix
    T12[:3, 3] = t_vector
    T12[3, :] = [0, 0, 0, 1]  # Ensure homogeneous structure

    return T12

def create_relative_transformation2(source_matrix, target_matrix):
    """Create a 4x4 transformation matrix from source frame to target frame."""
    source_inv = np.linalg.inv(source_matrix)
    return concat(source_inv, target_matrix)

# Define frames
world_real_frame = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

calibration_board_frame = np.array([
    [0, -1, 0, 0],
    [-1, 0, 0, -0.1],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
])

camera_frame = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
])

# Compute transformations
T_world_to_board = create_relative_transformation2(world_real_frame, calibration_board_frame)
T_board_to_camera = create_relative_transformation2(calibration_board_frame, camera_frame)

# Compute camera frame in world frame
camera_frame_computed = T_board_to_camera @ T_world_to_board

print(f'Computed Camera Frame:\n{camera_frame_computed}')

# Check for correctness with tolerance
assert np.allclose(camera_frame, camera_frame_computed, atol=1e-7), \
    "Computed camera frame does not match expected frame"
    
# Compute transformations
T_world_to_board = create_relative_transformation(world_real_frame, calibration_board_frame)
T_board_to_camera = create_relative_transformation(calibration_board_frame, camera_frame)
                                                    
# Compute camera frame in world frame
camera_frame_computed = T_board_to_camera @ T_world_to_board

print(f'Orth Computed Camera Frame:\n{camera_frame_computed}')

# Check for correctness with tolerance
assert np.allclose(camera_frame, camera_frame_computed, atol=1e-7), \
    "Computed camera frame does not match expected frame"
    
# Compute transformations
T_world_to_board = create_relative_transformation_orth_rotation(world_real_frame, calibration_board_frame)
T_board_to_camera = create_relative_transformation_orth_rotation(calibration_board_frame, camera_frame)
                                                    
# Compute camera frame in world frame
camera_frame_computed = T_board_to_camera @ T_world_to_board

print(f'Computed Camera Frame:\n{camera_frame_computed}')

# Check for correctness with tolerance
assert np.allclose(camera_frame, camera_frame_computed, atol=1e-7), \
    "Computed camera frame does not match expected frame"
