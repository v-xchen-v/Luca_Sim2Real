import numpy as np

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