import numpy as np

def load_camera_intrinsic_from_npy(npy_file_dir):
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

def load_table_to_camera_extrinsics_from_npy(npy_file):
    """
    Load extrinsic transformation (table to camera) from a YAML file.

    Parameters:
    - yaml_file: Path to the YAML file containing the extrinsics.

    Returns:
    - T: 4x4 transformation matrix (table to camera).
    """
    T = np.load(npy_file)
    return T