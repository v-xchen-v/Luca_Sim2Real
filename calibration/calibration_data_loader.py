import numpy as np
import os

TABLE_TO_CAMERA_EXTRINSIC_FILE = "table_to_camera.npy" # a 4x4 transformation matrix from table to camera

def table_to_camera_extrinsics_exist(npy_dir):
    """
    Check if the table-to-camera extrinsics exist.

    Parameters:
    - npy_file: Path to the folder contains the npy file containing the extrinsics.

    Returns:
    - True if the file exists, False otherwise.
    """
    npy_file = os.path.join(npy_dir, TABLE_TO_CAMERA_EXTRINSIC_FILE)
    return os.path.exists(npy_file)

def load_table_to_camera_extrinsics_from_npy(npy_dir):
    """
    Load extrinsic transformation (table to camera) from a npy file.

    Parameters:
    - npy_dir: Path to the folder contains the npy file containing the extrinsics.

    Returns:
    - T: 4x4 transformation matrix (table to camera).
    """
    npy_file = os.path.join(npy_dir, TABLE_TO_CAMERA_EXTRINSIC_FILE)
    T = np.load(npy_file)
    return T