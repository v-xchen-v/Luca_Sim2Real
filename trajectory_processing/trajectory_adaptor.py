import numpy as np
import json
from pytransform3d.transformations import plot_transform
import matplotlib.pyplot as plt
from calibration.calibration_precomputed_data_loader import load_camera_intrinsics_from_npy, load_eyehand_extrinsics_from_npy
from calibration.calibration_data_loader import load_table_to_camera_extrinsics_from_npy, table_to_camera_extrinsics_exist
import os
from calibration.calibrate_board_to_camera import capture_frame_and_save_table_calibration
from coordinates.frame_manager import FrameManager

"""
Steps:
    1. Load pre-computed calibration data
    2. Capture frame and compute table-to-camera transformation
    2. TODO: Compute relative transformation between frames based on calibration data
"""
# Dummy logic of trajectory adaptor
class TrajectoryAdaptor:
    def __init__(self):
        """
        Initialize TrajectoryAdaptor with pre-computed calibration data.
        
        Parameters:
        - calibration_data_dir: Path to the data folder (specfied camera and date) containing calibration data.
        """
        # calibration_data is a dictionary containing camera intrinsic and extrinsic parameters and eye-to-hand transformation
        self.calibration_data = {}
        
        # Initialize frame manager to manage coordinate frames and transformations between frames in the system
        self.frame_manager = FrameManager()
        ## Initialize frames with known names
        self.frame_names = [
            "calibration_board_real",
            "camera_real",
            "robot_base_real",
            "right_hand_base_real",
            # "world_real",
        ]
        self.frame_manager.initialize_frames(self.frame_names)
        
        # # Initialize transformations between calibration board, camera, robot base with calibration data
        # self.compute_all_transformations_with_calibration_data()
        
        
        self.camera_intrinsic = self.calibration_data.get("camera_intrinsic")
        self.T_camera_to_robot = np.array(self.calibration_data.get("T_camera_to_robot"))
        self.T_table_to_camera = None  # To be computed in dynamic calibration

    def get_calibration_data(self, calibration_data_dir, overwrite_if_exists=False, calibration_board_info=None, error_threshold=0.5):
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
        mtx, dist, T_camera_to_robot_base = self._load_precomputed_calibration_data(calibration_data_dir)
        self.calibration_data['camera_intrinsic'] = {
            'mtx': mtx,
            'dist': dist
        }
        self.calibration_data['T_camera_to_robot'] = T_camera_to_robot_base
        
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
        
        # load table calibration data
        table_calibration_data_exist = table_to_camera_extrinsics_exist(calibration_data_dir+"/table_to_camera")
        if table_calibration_data_exist:
            T_table_to_camera = load_table_to_camera_extrinsics_from_npy(calibration_data_dir+"/table_to_camera")
            self.calibration_data['T_table_to_camera'] = T_table_to_camera
        else:
            raise ValueError("Table calibration data not found.")
        
        return calibration_data
        
    def _load_precomputed_calibration_data(self, calibration_data_dir):
        """
        Load pre-computed calibration data from .npy files in the specified directory.
        """
        # check whether the calibration data directory exists
        if not os.path.exists(calibration_data_dir):
            raise ValueError("Calibration data directory not found.")
        
        mtx, dist = load_camera_intrinsics_from_npy(calibration_data_dir + '/camera_intrinsics')
        T_camera_to_robot_base = load_eyehand_extrinsics_from_npy(calibration_data_dir + '/camera_to_robot')
        return mtx, dist, T_camera_to_robot_base

    def compute_all_transformations_with_calibration_data(self):
        """
        Register transformations between calibration board, camera, robot base with calibration data.
        
        """
        # Ensure all necessary calibration data is available
        if "T_camera_to_robot" not in self.calibration_data:
            raise ValueError("Camera-to-Robot transformation not found. Load calibration data first.")
        if "T_table_to_camera" not in self.calibration_data:
            raise ValueError("Table-to-Camera transformation not found. Load calibration data first.")
        
        # Extract transformations from calibration data
        T_camera_real_to_robot = self.calibration_data["T_camera_to_robot"]
        T_calibration_board_real_to_camera = self.calibration_data["T_table_to_camera"]
        
        # Register transformations between frames
        self.frame_manager.add_transformation("calibration_board_real", "camera_real", T_calibration_board_real_to_camera)
        self.frame_manager.add_transformation("camera_real", "robot_base_real", T_camera_real_to_robot)
        
        print("Transformations between calibration board, camera, robot base added with calibration data.")
        
    def compute_relative_transformation(self, T_source, T_target):
        """
        Compute the relative transformation between two frames.
        T_source: 4x4 matrix representing the source frame.
        T_target: 4x4 matrix representing the target frame.
        """
        T_source_inv = np.linalg.inv(T_source)
        return T_source_inv @ T_target

    def map_sim_to_real(self, T_sim_world_to_table):
        """
        Map a transformation from the simulator to the real world.
        T_sim_world_to_table: 4x4 matrix of simulator’s table relative to simulator world.
        """
        if self.T_table_to_camera is None:
            raise ValueError("Table-to-Camera transformation not set. Perform calibration first.")

        # Compute real world mapping: camera-to-robot and table-to-camera are known
        T_table_to_robot = self.compute_relative_transformation(self.T_table_to_camera, self.T_camera_to_robot)
        print("Computed Table-to-Robot transformation:\n", T_table_to_robot)

        # Now compute the mapping from simulator to real world
        T_sim_to_real = T_table_to_robot @ T_sim_world_to_table
        return T_sim_to_real

    def visualize_frames(self, frames):
        """
        Visualize multiple frames in a 3D plot.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for name, T in frames.items():
            plot_transform(ax=ax, A2B=T, s=0.2, name=name)

        # Set plot limits and labels
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([0, 2])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        plt.title("Coordinate Frames Visualization")
        plt.show()

# Example Usage
if __name__ == "__main__":
    # Initialize the trajectory adaptor with pre-computed calibration data
    adaptor = TrajectoryAdaptor("/home/yichao/Documents/repos/Luca_Transformation/calibration/calibration_data/camera1")

    # # Capture frame and compute table-to-camera transformation (mock example)
    # T_table_to_camera = [
    #     [1, 0, 0, 0.5],
    #     [0, 1, 0, 0.3],
    #     [0, 0, 1, 0.2],
    #     [0, 0, 0, 1]
    # ]
    # adaptor.capture_frame_for_calibration(T_table_to_camera)

    # # Define the simulator world to table transformation (mock example)
    # T_sim_world_to_table = np.array([
    #     [0, -1, 0, 1],
    #     [1, 0, 0, 2],
    #     [0, 0, 1, 0],
    #     [0, 0, 0, 1]
    # ])

    # # Map simulator transformations to real-world coordinates
    # T_sim_to_real = adaptor.map_sim_to_real(T_sim_world_to_table)
    # print("Mapped Simulator to Real-World Transformation:\n", T_sim_to_real)

    # # Visualize the transformations
    # frames = {
    #     "Camera to Robot": adaptor.T_camera_to_robot,
    #     "Table to Camera": adaptor.T_table_to_camera,
    #     "Sim World to Table": T_sim_world_to_table,
    #     "Sim to Real": T_sim_to_real
    # }
    # adaptor.visualize_frames(frames)
