import numpy as np

def load_camera_intrinsics_from_npy(npy_file_dir):
    """
    Load the camera intrinsic matrix and distortion coefficients from a .npy file.
    
    Parameters:
    - npy_file_dir: Directory containing the .npy files.
    
    Returns:
    - mtx: Camera intrinsic matrix.
    - dist: Distortion coefficients.
    """
    
    mtx = np.load(npy_file_dir + '/mtx.npy')
    dist = np.load(npy_file_dir + '/dist.npy')
    return mtx, dist

def load_eyehand_extrinsics_from_npy(npy_file_dir):
    """
    Load the camera to robot matrix from a .npy file.
    
    Parameters:
    - npy_file_dir: Directory containing the .npy files.
    
    Returns:
    - T_camera_to_robot_base: 4x4 transformation matrix (camera to robot base).
    """
    
    T_camera_to_robot_base = np.load(npy_file_dir + 'cam2base_4x4.npy')
    return T_camera_to_robot_base