import os
from calibration.calibration_data_loader import (
    table_to_camera_extrinsics_exist,
    load_table_to_camera_extrinsics_from_npy
)

from calibration.calibration_precomputed_data_loader import (
    load_camera_intrinsics_from_npy, 
    load_eyehand_extrinsics_from_npy
)

from pytransform3d.transformations import invert_transform
from calibration.calibrate_board_to_camera import capture_frame_and_save_table_calibration

def _load_precomputed_calibration_data(calibration_data_dir):
    """
    Load pre-computed calibration data from .npy files in the specified directory.
    """
    # check whether the calibration data directory exists
    if not os.path.exists(calibration_data_dir):
        raise ValueError("Calibration data directory not found.")
    
    mtx, dist = load_camera_intrinsics_from_npy(calibration_data_dir + '/camera_intrinsics')
    T_robot_base_to_camera = load_eyehand_extrinsics_from_npy(calibration_data_dir + '/camera_to_robot')
    return mtx, dist, T_robot_base_to_camera

def get_calibration_data(calibration_data_dir, overwrite_if_exists=False, calibration_board_info=None, error_threshold=0.5):
    """
    Load calibration data, grab a camera frame and compute table calibration if necessary.
    
    Parameters:
    - calibration_data_dir: Path to the root folder containing calibration data.
    - overwrite_if_exists: Overwrite the existing table calibration data. False by default, use the previous table calibration data. 
                            Otherwise, capture a new frame to compute the table-to-camera transformation.
    - calibration_board_info: Information about the calibration board (rows, columns, square size). if using checkerboard, provide a dict as
                                {"pattern_size": (rows, columns), "square_size": size of a square on the checkerboard (in meters)}
    - error_threshold: Maximum allowable reprojection error when computing table calibration. if None, no error is raised.
    
    Returns:
    - calibration_data: A dictionary containing camera intrinsic and extrinsic parameters and eye-to-hand transformation. Example:
                        {
                            "camera_intrinsic": {
                                "mtx": camera_matrix,
                                "dist": distortion_coefficients
                            },
                            "T_camera_to_robot": camera-to-robot transformation matrix,
                            "T_table_to_camera": table-to-camera transformation matrix
                        }
    """
    # create a dictionary to store calibration data
    # keys: camera_intrinsic, T_camera_to_robot, T_table_to_camera
    calibration_data = {}
    
    # load precomputed calibration data
    mtx, dist, T_robot_base_to_camera = _load_precomputed_calibration_data(calibration_data_dir)
    calibration_data['camera_intrinsic'] = {
        'mtx': mtx,
        'dist': dist
    }
    calibration_data['T_camera_to_robot'] = invert_transform(T_robot_base_to_camera)
    
    # check whether table calibration need to be computed
    table_calibration_data_exist = table_to_camera_extrinsics_exist(calibration_data_dir+"/table_to_camera")
    if not table_calibration_data_exist or overwrite_if_exists:
        # check the calibration board information
        if calibration_board_info is None:
            raise ValueError("Calibration board information include pattern_size and square_size is required when table calibration need to be computed.")
        
        
        # capture a frame to compute table-to-camera transformation
        capture_frame_and_save_table_calibration(calibration_board_info['pattern_size'],
                                                    calibration_board_info['square_size'],
                                                    mtx,
                                                    dist, 
                                                    calibration_data_dir + '/table_to_camera',
                                                    error_threshold=error_threshold
                                                )

        # pop-up a window to show the calibration image to make sure the calibration physical setup is correct
        ## To avoid use wrong data, the calibration image should be checked by human.s
        _show_calibration_image(calibration_data_dir + "/table_to_camera")
        
    # load table calibration data
    table_calibration_data_exist = table_to_camera_extrinsics_exist(calibration_data_dir+"/table_to_camera")
    if table_calibration_data_exist:
        T_table_to_camera = load_table_to_camera_extrinsics_from_npy(calibration_data_dir+"/table_to_camera")
        calibration_data['T_table_to_camera'] = T_table_to_camera
        print(f"!!! Should check the table to camera translation are corrent in real setup: {T_table_to_camera[:3]}")
    else:
        raise ValueError("Table calibration data not found.")
    
    return calibration_data

def _show_calibration_image(calibration_data_dir):
    import cv2
    calibration_image_path = calibration_data_dir + "/corner_visualization.jpg"
    frame = cv2.imread(calibration_image_path)
    if frame is None:
        raise ValueError("Calibration image not found.")
    cv2.imshow("Calibration Image", frame)
    cv2.waitKey(0)