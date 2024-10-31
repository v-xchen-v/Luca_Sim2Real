# Transformation Matrices between frames

import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform, concat
from scipy.spatial.transform import Rotation as R

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
def create_relative_transformation(source_matrix, target_matrix):
    """
    Create a 4x4 homogeneous transformation matrix from a source frame to a target frame.

    Parameters:
    - source_frame: A 4x4 homogeneous transformation matrix representing the source frame.
    - target_frame: A 4x4 homogeneous transformation matrix representing the target frame.

    Returns:
    - T: A 4x4 homogeneous transformation matrix that transforms points from the source frame to the target frame.
    """
    # Ensure the matrices are valid 4x4 matrices
    if source_matrix.shape != (4, 4) or target_matrix.shape != (4, 4):
        raise ValueError("Both source_matrix and target_matrix must be 4x4 matrices.")

    # Check if the source matrix is invertible
    if np.linalg.det(source_matrix) == 0:
        raise ValueError("Source matrix is not invertible.")

    try:
        # Invert the source matrix to get the transformation from source to world
        source_inv = np.linalg.inv(source_matrix)

        # !!! Must use concat instead of np.dot to ensure the correct transformation. It waste me a whole day! !!!
        # # Compute the relative transformation from source to target
        # T_source_to_target = np.dot(source_inv, target_matrix)
        T_source_to_target = concat(source_inv, target_matrix)
        # Return the resulting 4x4 transformation matrix.
        return T_source_to_target

    except np.linalg.LinAlgError as e:
        print("Error during matrix inversion:", e)
        raise

# def compute_relative_transformation(self, T_source, T_target):
#     """
#     Compute the relative transformation between two frames.
#     T_source: 4x4 matrix representing the source frame.
#     T_target: 4x4 matrix representing the target frame.
#     """
#     T_source_inv = np.linalg.inv(T_source)
#     return T_source_inv @ T_target


def matrix_to_xyz_quaternion(matrix, quaternion_order="xyzw"):
    """
    Convert a 4x4 transformation matrix to a 7-element vector [x, y, z, qx, qy, qz, qw].

    Parameters:
    - matrix: A 4x4 homogeneous transformation matrix.

    Returns:
    - xyzxyzw: A 7-element vector representing the translation and rotation of the matrix.
    """
    # Extract the translation vector from the last column of the matrix.
    translation = matrix[:3, 3]

    # Extract the rotation matrix from the top-left 3x3 submatrix.
    rotation_matrix = matrix[:3, :3]

    # Convert the rotation matrix to a quaternion using pytransform3d.
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    if quaternion_order == "wxyz":
        quaternion = quaternion[[3, 0, 1, 2]]
        
    # Combine the translation and quaternion into a single 7-element vector.
    xyzq = np.concatenate([translation, quaternion])

    # Return the resulting 7-element vector.
    return xyzq