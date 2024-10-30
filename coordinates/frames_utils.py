import numpy as np
from scipy.spatial.transform import Rotation as R

# Helper function to create a transformation matrix with rotation and translation
def create_frame(origin, euler_angles):
    """
    Create a 4x4 transformation matrix for a frame with a given origin and rotation.
    
    Parameters:
    - origin: A array [x, y, z] representing the origin of the frame.
    - euler_angles: A array [roll, pitch, yaw] for the frame's rotation in radians.
    
    Returns:
    - A 4x4 transformation matrix.
    """
    # Set default values if not provided
    if origin is None:
        origin = [0, 0, 0]
    if euler_angles is None:
        euler_angles = [0, 0, 0]
        
    T = np.eye(4)  # Initialize 4x4 identity matrix
    T[:3, :3] = R.from_euler('XYZ', euler_angles).as_matrix()  # Apply rotation
    T[:3, 3] = origin  # Set the origin (translation)
    return T