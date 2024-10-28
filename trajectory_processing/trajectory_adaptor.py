import numpy as np
import json
from pytransform3d.transformations import plot_transform
import matplotlib.pyplot as plt
from calibration.calibration_precomputed_data_loader import load_camera_intrinsics_from_npy, load_eyehand_extrinsics_from_npy
from calibration.calibration_data_loader import load_table_to_camera_extrinsics_from_npy, table_to_camera_extrinsics_exist
import os
from calibration.calibrate_board_to_camera import capture_frame_and_save_table_calibration
from coordinates.frame_manager import FrameManager
from coordinates.transformation_utils import create_transformation_matrix, create_relative_transformation
from scipy.spatial.transform import Rotation as R
from pytransform3d.transformations import concat, invert_transform
from pytransform3d.transformations import (     transform_from_pq, pq_from_transform, concat )
"""
Steps:
    1. Load pre-computed calibration data
    2. Capture frame and compute table-to-camera transformation
    3. Compute relative transformation between frames based on calibration data
    4. Locate the object in the real world by relative pos to calibration board.
        - Option1 - Dummy Setup: Manually put the object on the calibration board's origin (or with translation, but origin is less effort for test) 
            then set relative rotation.
        - Option2 - Use ICP: Compute the object's position in camera coordinate.
    5. Compute the transformation between the object and the robot base.
    6. Map simulator transformations to real-world coordinates with contraints:
        - the transformation between object and right_hand_base should be the same in both simulator and real world.
    7. Compute transformation between robot_right_hand_base to robot_base in real world.

# Option1:
# Step real world
# 1. Load pre-computed calibration data
# 2. Capture frame and compute table-to-camera transformation
# 3. Compute relative transformation between frames based on calibration data
# 4. (Manually) Put the object on the calibration board's origin.
# Step sim

# Map them
# 5. Compute relative transformation based on 4.
        
Map simulator transformations to real-world coordinates by setting the transformation between real_world(same as sim_world)
and calibration_board_real as the identity matrix.
1. 
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
        ### Register the frames in this system
        self.frame_names = [
            # Real world setup
            ## calibration transformations
            "calibration_board_real",
            "camera_real",
            "robot_base_real",
            
            ## readable purpose
            "readable_real",
            
            ## object setup
            "object_real",
            
            ## 
            # sim2real by contraints on object and right hand base and readable sim_trajectory
            ## initial pose
            "object_sim",
            "sim_world",
            "right_hand_base_sim",
            
            # bridge sim and real by 'object_sim' and 'object_real'
        
            ## from step 0 to step n    
            "right_hand_base_step0_real",
            
            ## 
            "right_hand_base_real",
            # "real_world",
            # "sim_world",
        ]
        self.frame_manager.initialize_frames(self.frame_names)
        
        # # Initialize transformations between calibration board, camera, robot base with calibration data
        # self.compute_all_transformations_with_calibration_data()
        
        
        self.camera_intrinsic = self.calibration_data.get("camera_intrinsic")
        self.T_camera_to_robot = np.array(self.calibration_data.get("T_camera_to_robot"))
        self.T_table_to_camera = None  # To be computed in dynamic calibration

    def add_transfromations_with_calibration(self):
        self._compute_transformations_with_calibration_data()
        
        # check the transformation based on calibration data is computed
        ## List the frames that transformations between them should already known.
        transform_added_frames = ["calibration_board_real", "camera_real", "robot_base_real"] 
        ## Check the known transformations are computed
        # if not self.frame_manager.frames_connected(transform_added_frames):
        #     raise ValueError("Transformations between calibration board, camera, robot base not computed. Compute them first.")

    def object_setup(self):
        pass
    
    def map_sim_to_real(self):
        """Load simulated trajectory contraints and apply to real world"""
        pass
        
    def save_adapted_trajectory(self):
        pass
    
    def _show_calibration_image(self, calibration_data_dir):
        import cv2
        calibration_image_path = calibration_data_dir + "/corner_visualization.jpg"
        frame = cv2.imread(calibration_image_path)
        if frame is None:
            raise ValueError("Calibration image not found.")
        cv2.imshow("Calibration Image", frame)
        cv2.waitKey(0)
        
    def _get_calibration_data(self, calibration_data_dir, overwrite_if_exists=False, calibration_board_info=None, error_threshold=0.5):
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
        mtx, dist, T_robot_base_to_camera = self._load_precomputed_calibration_data(calibration_data_dir)
        self.calibration_data['camera_intrinsic'] = {
            'mtx': mtx,
            'dist': dist
        }
        self.calibration_data['T_camera_to_robot'] = invert_transform(T_robot_base_to_camera)
        
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
            self._show_calibration_image(calibration_data_dir + "/table_to_camera")
            
        # load table calibration data
        table_calibration_data_exist = table_to_camera_extrinsics_exist(calibration_data_dir+"/table_to_camera")
        if table_calibration_data_exist:
            T_table_to_camera = load_table_to_camera_extrinsics_from_npy(calibration_data_dir+"/table_to_camera")
            self.calibration_data['T_table_to_camera'] = T_table_to_camera
            print(f"!!! Should check the table to camera translation are corrent in real setup: {T_table_to_camera[:3]}")
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
        T_robot_base_to_camera = load_eyehand_extrinsics_from_npy(calibration_data_dir + '/camera_to_robot')
        return mtx, dist, T_robot_base_to_camera
    
    def _compute_transformations_with_calibration_data(self):
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
        # self.frame_manager.add_transformation("camera_real", "robot_base_real", T_camera_real_to_robot) # wired, wrong
        # self.frame_manager.add_transformation("calibration_board_real", "robot_base_real", T_calibration_board_real_to_camera @ T_camera_real_to_robot) # wired, right
        T_calibration_board_to_robot_base = T_calibration_board_real_to_camera @ T_camera_real_to_robot
        self.frame_manager.add_transformation(
            "camera_real", 
            "robot_base_real",
            create_relative_transformation(T_calibration_board_real_to_camera, T_calibration_board_to_robot_base)
        )
        
        
        print("Transformations between calibration board, camera, robot base added with calibration data.")
    
    def _compute_transformations_with_dummy_object_setup(self, translation_object_to_world, rotation_object_to_world=None):
        """
        Aim to compute the transformation between the object and the robot base with the dummy object setup.
        Dummy object setup: 
        1. Put the object on the calibration board's origin.
        2. Set the real world frame same as the simulator world frame.
        3. Compute the transformation between the object and the robot base.????
        
        Parameters:
        - translation_object_to_world: 3D translation vector of the object relative to the world frame.
        - rotation_object_to_world: 3x3 rotation matrix of the object relative to the world frame.
        
        Returns:
        - T_object_real_to_robot: 4x4 transformation matrix for the object relative to the robot base.
        """
        # Define the world's transformation same origin with the calibration board but rotate.
        ## world(source) frame
        world_real_frame = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        ## calibartion board frame,
        calibration_board_frame = np.array([[0, -1, 0, 0], # target x is source -y
                                            [-1, 0, 0, 0], # target y is source -x
                                            [0, 0, -1, 0], # target z is source -z
                                            [0, 0, 0, 1]])
        self.frame_manager.add_transformation(
            "real_world", "calibration_board_real",
            create_relative_transformation(world_real_frame, calibration_board_frame))
        
        # Define the real world's transformation same as the simulator world
        self.frame_manager.add_transformation("real_world", "sim_world", np.eye(4))
        
        # Define the object's transformation relative to the real board with the dummy manually setup
        T_object_real_to_world = create_transformation_matrix(translation_object_to_world, rotation_object_to_world)    
        
        # Register the transformation between the object and the real world    
        self.frame_manager.add_transformation("object_real", "real_world", T_object_real_to_world)
        
        # Compute the transformation between the object and the robot base
        T_object_real_to_robot = self.frame_manager.get_transformation("object_real", "robot_base_real")
        
        # Register the transformation between the object and the robot base
        self.frame_manager.add_transformation("object_real", "robot_base_real", T_object_real_to_robot)
        
        # # Compute the transformation between the object and the robot base
        # T_object_to_robot = self._compute_object_to_robot_transformation(T_object_real_to_calibration_board_real)
        
        # # Register the object's transformation
        # self.frame_manager.add_transformation("object_real", "robot_right_hand_real", T_object_to_robot)
        
        print("Transformations between object and robot base added with dummy object setup.")
        return T_object_real_to_robot
        
        
    # def _compute_object_to_robot_transformation(self, T_object_to_board):
    #     """
    #     When the relative of object to board is known, compute the transformation between the object and the robot base.

    #     Parameters:
    #     - T_object_to_board: 4x4 transformation matrix for the object relative to the calibration board.
        
    #     Returns:
    #     - T_object_to_robot: 4x4 transformation matrix for the object relative to the robot base.
    #     """
    #     if self.T_table_to_camera is None:
    #         raise ValueError("Table-to-Camera transformation not set. Perform calibration first.")
        
    #     # Compute the transformation between the object and the camera
    #     T_object_to_camera = create_relative_transformation(self.T_table_to_camera, T_object_to_board)
        
    #     T_object_to_robot = (
    #         self.T_camera_to_robot @
    #         self.T_board_to_camera @
    #         T_object_to_board
    #     )
        
    #     return T_object_to_robot
        
    # def compute_relative_transformation(self, T_source, T_target):
    #     """
    #     Compute the relative transformation between two frames.
    #     T_source: 4x4 matrix representing the source frame.
    #     T_target: 4x4 matrix representing the target frame.
    #     """
    #     T_source_inv = np.linalg.inv(T_source)
    #     return T_source_inv @ T_target
    def _parse_sim_trajectory_file(self, traj_npy_file):
         # Read the trajectory file from the simulator
        traj_sim_data = np.load(traj_npy_file, allow_pickle=True)
        print(traj_sim_data.item().keys())
        
        # indexed joints information
        # # [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15]
        # # 'move_x': 0
        # # 'move_y': 1
        # # 'move_z': 2
        # # 'rot_r': 3
        # # 'rot_p': 4
        # # 'rot_y': 5
        # # 'R_index_proximal_joint': 6
        # # 'R_middle_proximal_joint': 7
        # # 'R_pinky_proximal_joint': 8
        # # 'R_ring_proximal_joint': 9
        # # 'R_thumb_proximal_yaw_joint':10
        # # 'R_thumb_proximal_pitch_joint': 15
        
        
        # Extract the hand joint positions from the trajectory
        hand_joint_pos = traj_sim_data.item()['hand_dof_pos_buf'] # [num_steps, 1, num_joints]
        driven_hand_dof_index = [8, 9, 7, 6, 15, 10]
        driven_hand_joint_pos_sim = hand_joint_pos[:, 0, driven_hand_dof_index] # [num_steps, 1, num_driven_joints]
        driven_hand_joint_pos_sim = driven_hand_joint_pos_sim.squeeze(axis=1) # [num_steps, num_driven_joints]
        # traj_sim_data['driven_hand_joint_pos'] = driven_hand_joint_pos
        
        
        # Extract the base link(right hand base) position from the trajectory
        ## [x, y, z, qx, qy, qz, qw] for translation and quaternion rotation
        right_hand_base_sim = traj_sim_data.item()['right_hand_base_pose_buf'] # [num_steps, 1, 7]
        right_hand_base_sim = right_hand_base_sim.squeeze(axis=1) # [num_steps, 7]
        
        # # Extract the grasp flag from the trajectory
        # grasp_flag = traj_sim_data.item()['hold_flag_buf'] # [num_steps, 1, 1]
        # grasp_flag.squeeze(axis=1) # [num_steps, 1]
        
        # Extract the object position from the trajectory
        object_pos = traj_sim_data.item()['object_pose_buf'] # [num_steps, 1, 7]
        object_pos = object_pos.squeeze(axis=1) # [num_steps, 7]
        
        # Extract the grasp flag from the trajectory
        grasp_flag_sim = traj_sim_data.item()['hold_flag_buf'] # [num_steps, 1]
        return driven_hand_joint_pos_sim, right_hand_base_sim, object_pos, grasp_flag_sim
    
    def _build_transformations_object_hand_world_at_initial(self, traj_npy_file):
        """Build the transformation between object and right hand base, object and world at initial step."""
        _, right_hand_base_sim, object_pos, _ = self._parse_sim_trajectory_file(traj_npy_file)
         
        # In the first step, compute relative transformation of object to right hand base
        object_pos_0 = object_pos[0, :]
        right_hand_base_pos_0 = right_hand_base_sim[0, :]
        ## convert [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix
        # object_in_world_frame0 = create_transformation_matrix(object_pos_0[:3], R.from_quat(object_pos_0[3:][[1, 2, 3, 0]]).as_matrix())
        object_in_world_frame0 = transform_from_pq(object_pos_0)
        
        # Once we know the first frame object frame in world, should got the modeling axes of object in world
        # Despite of sim_traj file, we setting the sim world frame as same position as object but object could upper on z-axis and have different orientation.
        # So, we should know the object's coordinate frame in the world coordinate frame in sim.
        # It's reasonable because the x, y offset of object in the sim not matters for grasp, since we know the relative pos of object to right hand base.
        object_pos_0_z = object_pos_0[2]
        object_pos_0_wxyz = object_pos_0[3:]
        object_pos_0_in_sim_world_rotation_matrix = R.from_quat(object_pos_0_wxyz[[1,2,3,0]]).as_matrix()
        T_object_in_world_sim = create_transformation_matrix(translation=[0, 0, object_pos_0_z], 
                                                             rotation_matrix=object_pos_0_in_sim_world_rotation_matrix)
        self.frame_manager.add_transformation("sim_world", "object_sim", object_in_world_frame0)
        
        # worl
        right_hand_base_in_world_frame0 = transform_from_pq(right_hand_base_pos_0)
        # self.frame_manager.add_transformation("right_hand_base_sim", "sim_world", invert_transform(right_hand_base_in_world_frame0))

        # right_hand_base_in_world_frame0 = create_transformation_matrix(right_hand_base_pos_0[:3], R.from_quat(right_hand_base_pos_0[3:][[1, 2, 3, 0]]).as_matrix())
        # compute relative transformation between object and right hand base
        # T_right_hand_base_sim_to_object = concat(invert_transform(right_hand_base_in_world_frame0), object_in_world_frame0)
        # self.frame_manager.add_transformation("right_hand_base_sim", "object_sim", T_right_hand_base_sim_to_object)
        self.frame_manager.add_transformation("sim_world", "right_hand_base_sim", right_hand_base_in_world_frame0)
        pass
        
    def parse_sim_trajectory(self, traj_npy_file):
        """Parse the trajectory from the simulator and compute the relative transformation between the object to right hand base.
        
        Build up:
        - Object coordinate frame in the sim world which used in mapping object in real world.
        - Right hand base relative pos to object at initial.
            - T_right_hand_base_sim_to_object: 4x4 matrix of simulator's right hand base relative to object at step 0.
        
        Returns:
        - driven_hand_joint_pos_sim: [num_steps, num_driven_joints] array of driven hand joint positions, in radias.
        - right_hand_pos_sim: [num_steps, 4, 4] array of right hand base positions 4x4 matrix.
        - graso_flag_sim: [num_steps, 1] array of grasp flags.
        # - traj_sim_data: Dictionary containing the parsed trajectory data."""

        
        self._build_transformations_object_hand_world_at_initial(traj_npy_file)
                
        # load data from file
        driven_hand_joint_pos_sim, right_hand_base_in_world_sim, object_pos, grasp_flag_sim = self._parse_sim_trajectory_file(traj_npy_file)
        
        ## convert the [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix
        right_hand_base_in_world_sim = np.array([create_transformation_matrix(pos[:3], R.from_quat(pos[3:][[1, 2, 3, 0]]).as_matrix()) for pos in right_hand_base_in_world_sim]) # [num_steps, 4, 4]
        
        # Return the relative transformation
        return driven_hand_joint_pos_sim, right_hand_base_in_world_sim, object_pos, grasp_flag_sim

    def compute_constrained_object_relative_to_right_hand_base(self):
        """
        Aim to compute the right hand base to robot base transformation in real world.
        
        Map sim to real by containing the transformation between object and right_hand_base the same in both simulator and real world.
        Known the transformation between the object and the right-hand base in the simulator and object to robot base in real world. 
        Compute the right hand base position in real world constrained to maintain the same relative position between the object and the right-hand base in both real and simulated worlds.
        
        Parameters:
        - T_object_to_right_hand_base: 4x4 matrix of simulator's right hand base relative to object.
        
        Returns:
        - T_right_hand_base_real_to_robot_base: 4x4 matrix of real world's right hand base relative to robot base.
        """
        # # check the should known transformations
        # if self.frame_manager.get_transformation("object_real", "real_world") is None:
        #     raise ValueError("Object to real world transformation not found. Perform object setup first.")
        
        # if self.frame_manager.get_transformation("right_hand_base_sim", "object_sim") is None:
        #     raise ValueError("Right hand base to object transformation in simulator not found.")
        
        # check the input transformation
        T_right_hand_base_sim_to_object = self.frame_manager.get_transformation("right_hand_base_sim", "object_sim")
        if T_right_hand_base_sim_to_object is None:
            raise ValueError("Transformation between right hand base and object in simulator is required.")
        
        self.frame_manager.add_transformation("right_hand_base_step0_real", "object_real", T_right_hand_base_sim_to_object)
        T_right_hand_base_real_to_robot_base = self.frame_manager.get_transformation("right_hand_base_step0_real", "robot_base_real")
        if T_right_hand_base_real_to_robot_base is None:
            raise ValueError("Right hand base to robot base transformation in real world not found.")
        
        return T_right_hand_base_real_to_robot_base
        
        # if self.T_table_to_camera is None:
        #     raise ValueError("Table-to-Camera transformation not set. Perform calibration first.")

        # # Compute real world mapping: camera-to-robot and table-to-camera are known
        # T_table_to_robot = self.compute_relative_transformation(self.T_table_to_camera, self.T_camera_to_robot)
        # print("Computed Table-to-Robot transformation:\n", T_table_to_robot)

        # # Now compute the mapping from simulator to real world
        # T_sim_to_real = T_table_to_robot @ T_sim_world_to_table
        # return T_sim_to_real

    def compute_right_hand_base_to_object_steps_in_sim(self, right_hand_base_in_world_sim, object_pos):
        """
        Known the relative pos between object and right hand base at initial in simulator, 
        And Given the right hand base positions on world coordinate in simulator,
        compute the relative pos between object and right hand base at each step.
        
        Parameters:
        - right_hand_base_pose_sim: [num_steps, 4, 4] array of right hand base positions 4x4 matrix.
        
        Returns:
        - T_right_hand_base_steps_to_object_in_sim: [num_steps, 4, 4] array of right hand base to object positions 4x4 matrix
        """
        
        # _, right_hand_base_in_world_sim, object_pos, _ = self._parse_sim_trajectory_file(traj_npy_file)
         
        def compute_right_hand_base_to_object_stepi_in_sim(right_hand_base_in_world_sim, object_pos, step_i):
            # In the first step, compute relative transformation of object to right hand base
            object_pos_0 = object_pos[0, :]
            right_hand_base_in_world_frame0 = right_hand_base_in_world_sim[step_i, :]
            ## convert [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix
            # object_in_world_frame0 = create_transformation_matrix(object_pos_0[:3], R.from_quat(object_pos_0[3:][[1, 2, 3, 0]]).as_matrix())
            object_in_world_frame0 = transform_from_pq(object_pos_0)
            
            # TODO: seems weird here.
            # right_hand_base_in_world_frame0 = create_transformation_matrix(right_hand_base_frame_0[:3], R.from_quat(right_hand_base_frame_0[3:][[1, 2, 3, 0]]).as_matrix())
            ## compute relative transformation between object and right hand base
            T_right_hand_base_sim_to_object = concat(invert_transform(right_hand_base_in_world_frame0), object_in_world_frame0)
            # T_right_hand_base_sim_to_object = concat(object_in_world_frame0, invert_transform(right_hand_base_in_world_frame0))
            # T_right_hand_base_sim_to_object = concat(invert_transform(right_hand_base_in_world_frame0), object_in_world_frame0)
            return T_right_hand_base_sim_to_object
            
        T_right_hand_base_steps_to_object_in_sim = [compute_right_hand_base_to_object_stepi_in_sim(right_hand_base_in_world_sim, object_pos, i) for i in range(len(right_hand_base_in_world_sim))]
        return T_right_hand_base_steps_to_object_in_sim
            
        
        # # get known relative pos between object and right hand base in simulator
        # T_right_hand_base_to_object_sim_at_intial = self.frame_manager.get_transformation("right_hand_base_sim", "object_sim")
        # if T_right_hand_base_to_object_sim_at_intial is None:
        #     raise ValueError("Transformation between right hand base and object in simulator is required.")
        
        # T_right_base_hand_to_world_sim = [invert_transform(T) for T in right_hand_base_in_world_sim]
        # # compute relative transformation between each step with the first step
        # T_right_hand_base_stepi_to_step0_in_sim = [
        #     # X @ stepi_to_world  = stepi_to_world, then X = world_to_step0
        #     # create_relative_transformation(T_right_base_hand_stepi_to_world, T_right_base_hand_to_world_sim[0])
        #     concat(T_right_base_hand_stepi_to_world, invert_transform(T_right_base_hand_to_world_sim[0]))
        #     for T_right_base_hand_stepi_to_world in T_right_base_hand_to_world_sim]
        
        # # Assume object is stable just as the first step, and the hand moves, computes the relative transformation between object and right hand base
        # # The assumption is reasonable, because we are doing sim2real for an open loop rl, assumed the object is stable during approach and affordance.
        # # T_right_hand_base_step0_to_object_in_sim = self.frame_manager.get_transformation("right_hand_base_sim", "object_sim")
        # T_right_hand_base_steps_to_object_in_sim = [concat(T_right_hand_base_to_object_sim_at_intial, invert_transform(T)) for T in T_right_hand_base_stepi_to_step0_in_sim]
        # # T_right_hand_base_steps_to_object_in_sim = [concat(T, T_right_hand_base_to_object_sim_at_intial) for T in T_right_hand_base_stepi_to_step0_in_sim]
        
        
        # # Got the relative pos between object and right hand base in sim at each step
        # return T_right_hand_base_steps_to_object_in_sim
        
    # def _locate_translation_object_in_readable_frame_in_real(self, translation_object_in_readable=[0, 0, 0]):
    #     """
    #     Locate the object's position(xyz only, no orientation) in the real world by setting 
    #     the object translate to the readable's origin(which is also the readable frame's origin).
    #     """
    #     # Assume the object is placed on the calibration board's origin        
    #     # Register the translation between the object and the readable frame
    #     tvec_object_in_readable = np.array(translation_object_in_readable)
    #     return tvec_object_in_readable
        
    # def _locate_rotation_object_in_readable_frame_in_real(self, rmatrix_object_readable_in_world_readable):
    #     """Q: given object_readable_in_world_readable, how to compute the rotation of object_sim_in_world_sim?"""
    #     """Set object in real have same orientation as readable frame with no rotation for human readable."""
    #     return rmatrix_object_readable_in_world_readable
    
    def _map_sim_to_real_hand_base(self):
        """hand base relative to object in real should as same as in sim."""
        
        # got object in readable frame in real
        
        ## 
        
    def _bridge_real_sim_with_object(self, T_right_hand_base_steps_to_object_in_sim):
        """What's mean mapping real objec to sim object?
        if object in real rotate, the sim object should rotate with it, so that the hand base with also rotate with it to keep the relative pos.
        """
        # Logic of mapping real object to sim object
        ## A(world), B(object), C(hand base)
        ## B2C known and keep same in real and sim, so that, B->B' = C->C'
        ## bind world so that object connect with rela pos, so that bind sim_world to 
        ## real_world frame which could be camera frame or calibration board or any frame known relative pos to object_real
        self.frame_manager.add_transformation("sim_world", "readable_real", np.eye(4))
        
        # A stands for world_to_object, A' stands for world_to_object in real, and T_A_Ap is computed as below: 
        T_A_changed = create_relative_transformation(
            self.frame_manager.get_transformation("sim_world", "object_sim"), 
            self.frame_manager.get_transformation("readable_real", "object_real"))
        
        # should same as T_right_hand_base_steps_to_object_in_real, but issue now
        # # T_A_Ap * T_A_B * T_Ap_A = T_Ap_Bp
        # T_right_hand_base_steps_to_object_in_real2 = [
        #     invert_transform(
        #     # T_A_changed @ invert_transform(T) @ invert_transform(T_A_changed))
        #     invert_transform(T) @ invert_transform(T_A_changed))
        #     for T in T_right_hand_base_steps_to_object_in_sim
        # ]
         
        T_real_world_to_object_real = self.frame_manager.get_transformation("readable_real", "object_real")
        T_sim_world_to_object_sim = self.frame_manager.get_transformation("sim_world", "object_sim")
        T_right_hand_base_steps_to_object_in_real = [
            create_relative_transformation(
                T_A_changed @ invert_transform(T) @ T_sim_world_to_object_sim,
                T_real_world_to_object_real)
            for T in T_right_hand_base_steps_to_object_in_sim
        ]
        
        return T_right_hand_base_steps_to_object_in_real        
    
    def _map_real_robot_action_to_sim(self, T_right_hand_base_steps_to_object_in_sim, 
            T_object_sim_to_object_real, # B2B',
            ):
        """Map the real robot action to the simulator."""
        # Given the relative pos of hand and object in sim, same relative pos in real should be kept. 
        # T_right_hand_base_steps_to_object_in_real = [invert_transform(concat
        #                                              (invert_transform(T_readable_real_to_object_real),
        #                                               concat(
        #                                                     T_world_sim_to_object_sim, invert_transform(T)
        #                                                     )))
        #                                              for T in T_right_hand_base_steps_to_object_in_sim]
        T_sim_world_to_object = self.frame_manager.get_transformation("sim_world", "object_sim")
        T_right_hand_base_steps_to_object_in_real = [
           invert_transform(concat(T_object_sim_to_object_real, invert_transform(T)))
                                                for T in T_right_hand_base_steps_to_object_in_sim]
        
        # # (A2B*B*B').T *(A'2B'*B'2B*B2C)
        # ## (A2B*B*B').T
        # T_world_sim_to_object_real = invert_transform(concat(T_world_sim_to_object_sim, T_object_sim_to_object_real))
        # ## A'2B'*B'2B
        # T_readable_real_to_object_sim = concat(T_readable_real_to_object_real, invert_transform(T_object_sim_to_object_real))
        # T_right_hand_base_steps_to_object_in_real = [
        #     invert_transform(
        #             concat(T_world_sim_to_object_real,
        #                 concat(T_readable_real_to_object_sim, invert_transform(T))
        #             )
        #         )
        #         for T in T_right_hand_base_steps_to_object_in_sim]
        return T_right_hand_base_steps_to_object_in_real
        # R_real_to_sim = T_real_to_sim[:3, :3]
        # # t_real_to_sim = T_real_to_sim[:3, 3]
        
        # T_right_hand_base_steps_to_object_in_real = []
        # for T_right_hand_base_stepi_to_object_in_sim in T_right_hand_base_steps_to_object_in_sim:
        #     # T_right_hand_base_stepi_to_object_in_sim C2B
        #     T_B2C = invert_transform(T_right_hand_base_stepi_to_object_in_sim)
        #     # Compute the new translation from object to right hand base
        #     t_B_prime_to_C = R_real_to_sim @ T_B2C[:3, 3]
            
        #     # compute the new rotation from object to right hand base
        #     R_B_prime_to_C = R_real_to_sim @ T_B2C[:3, :3]
            
        #     # Assume the new transformation between object and right hand base in real world
        #     B_prime2C = np.eye(4)
        #     B_prime2C[:3, :3] = R_B_prime_to_C
        #     B_prime2C[:3, 3] = t_B_prime_to_C
            
        #     T_right_hand_base_steps_to_object_in_real.append(invert_transform(B_prime2C))
            
            
        #     # when object_sim is np.eye, compute hand to object
            
            
        # return T_right_hand_base_steps_to_object_in_real
            
        
        # Compute right hand base to object in real world
        
        # # Adding a object readable in real to make object readable for human
        
        
        
        # # So that object in sim and object in real have different orientation.
        # # object_sim in world_sim should same as object_real in readable_real after considered rotation of object_sim_local and object_real_local of rotation
        # rmatrix_object_sim_to_world_sim = self.frame_manager.get_transformation("object_sim", "sim_world")[:3, :3]
        
        # # If the object axes is same as sim world
        
        # # Default(if object no rotation) object is same as sim world, since sim world is same as readable world so that object is readable for human too.
        # rmatrix_object_sim_to_object_readable_real = rmatrix_object_sim_to_world_sim
        
        # rmatrix_object_readable_in_sim_world = rmatrix_object_readable_in_world_readable
        # rmatrix_object_readable_to_sim_world = invert_transform(rmatrix_object_readable_in_sim_world)
        
        # # If rotate object in real world
        # rmatrix_object_sim_to_world_sim = concat(rmatrix_object_sim_to_object_real, rmatrix_object_readable_to_sim_world)
        
        # Assume that sim world, readable frame in real and object_readable_in_real have same axes default direction.
        # rmatrix_object_sim_to_object_real = rmatrix_object_sim_to_world_sim # so that object real is object sim rotate to 
        
        # got from input
        # rmatrix_object_readable_real_to_world_readable_real = invert_transform(rotation_object_in_readable)
        
        
    # def _map_real_to_sim_robot_base(self, translation_object_in_readable):
    #     """What's mean mapping real objec to sim object
    #     if object in real have translation or rotation, the robot should move or rotate to keep the relative pos.
    #     """
        
        
        
    # def _compute_right_hand_base_steps_to_object_base_real(self, T_right_hand_base_steps_to_object_in_real):
    #     # Known the relative pos between object and right hand base at each step in real world
    #     # Known the robot base relative to object_real
        
        
    #     T_right_hand_base_to_object_at_initial_sim = self.frame_manager.get_transformation("right_hand_base_sim", "object_sim")
    #     T_right_hand_base_to_object_at_initial_real = T_right_hand_base_to_object_at_initial_sim
    #     # Compute the relative pos between right hand base and right arm base in real world
    #     T_right_hand_base_steps_to_robot_base_real = [concat(T, T_right_hand_base_to_object_at_initial_real) for T in T_right_hand_base_steps_to_object_in_real]
    #     return T_right_hand_base_steps_to_robot_base_real
        
        
    def compute_right_hand_base_to_object(self, T_right_hand_base_steps_to_object_in_real):
        """Compute the transformation between the right hand base and the object in real world of num_steps 
        based on:
        - The relative pos of object to right hand base in simulator.
        - The relative pos of right hand base to right hand base in step 0 in simulator.
        
        Parameters:
        - right_hand_base_pose_sim: [num_steps, 4, 4] array of right hand base positions 4x4 matrix.
        
        Returns:
        - T_right_hand_base_steps_sim_to_object: [num_steps, 4, 4] array of right hand base to object positions 4x4 matrix
        """
        
        # compute relative transformation between each step with the first step

        
        # Assume object is stable just as the first step, and the hand moves, computes the relative transformation between object and right hand base
        T_right_hand_base_step0_sim_to_object = self.frame_manager.get_transformation("right_hand_base_step0_real", "object_real")
        T_object_step0_sim_to_robot_base = self.frame_manager.get_transformation("object_real", "robot_base_real")
        # self.frame_manager.visualize_transformations([("readable_real", "object_real"), ("object_real", "right_hand_base_step0_real"),
        #                                               ("right_hand_base_step0_real", "robot_base_real")])
        
        T_right_hand_base_steps_sim_to_object = [concat(T, T_right_hand_base_step0_sim_to_object) for T in T_right_hand_base_steps_relative_to_step0_in_sim]
        T_robot_base_to_right_hand_base_steps_sim = [invert_transform(concat(T, T_object_step0_sim_to_robot_base)) for T in T_right_hand_base_steps_sim_to_object]
        # T_right_hand_base_steps_sim_to_robot_base = [concat(T, T_object_step0_sim_to_robot_base) for T in T_right_hand_base_steps_sim_to_object]
        return T_right_hand_base_steps_sim_to_object, T_robot_base_to_right_hand_base_steps_sim
        
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
# if __name#__ == "__main__":
    # Initialize the trajectory adaptor with pre-computed calibration data
    # adaptor = TrajectoryAdaptor("/home/yichao/Documents/repos/Luca_Transformation/calibration/calibration_data/camera1")

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
