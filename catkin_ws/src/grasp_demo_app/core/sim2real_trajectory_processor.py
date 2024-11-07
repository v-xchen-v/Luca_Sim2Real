# Add module path if necessary
import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)


from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
import numpy as np
import json
from .object_manager import ObjectManager
from pytransform3d.transformations import invert_transform

class Sim2RealTrajectoryProcessor:
    def __init__(self, config) -> None:
        # Initialize attributes
        self.real_traj_adaptor = TrajectoryAdaptor()
        
        # Load configuration
        self._load_config(config)
        ## Camera settings
        ## self.camera_name = "camera1"
        ## self.capture_new_calibration = True
        self.camera_name = self.config["camera_name"]
        self.capture_new_calibration = self.config["capture_new_calibration"]
        
        ## Calibration board settings
        # self.calibration_board_pattern_size = (5, 8)
        # self.calibration_board_square_size = 0.03
        self.calibration_board_pattern_size = tuple(self.config["calibration_board_pattern_size"])
        self.calibration_board_square_size = self.config["calibration_board_square_size"]
        
        self.object_manager = ObjectManager()
        
        # Intermediate variables
        ## for t_scaled pre-grasp position
        self.T_base_to_hand_at_first_point = None
        
    def _load_config(self, config=None):
        # Load configuration from a dictionary or a json file
        if isinstance(config, str):
            with open(config, 'r') as f:
                self.config = json.load(f)
        elif isinstance(config, dict):
            self.config = config
        else:
            raise ValueError("Invalid config type. Use a dictionary or a json file.")
                
    def setup_robot_table(self):
        # Calculate the transformation between arm, table, and robot
        calibration_data_dir = f"calibration/calibration_data/{self.camera_name}"
        self.real_traj_adaptor.calculate_arm_table_robot_transform(
            calibration_data_dir=calibration_data_dir,
            overwrite_if_exists=self.capture_new_calibration,
            calibration_board_info={
                "pattern_size": self.calibration_board_pattern_size,
                "square_size": self.calibration_board_square_size
            }
        )
        
        
    def configure_object_settings(self, object_identifier):
        #  # Object-specific configurations
        # self.object_configs = {
        #     "names": [
        #         'orange_1024', 
        #         'coke_can_1030', 
        #         'realsense_box_1024',
        #         'cube_055_1103', 
        #         'bottle_coconut_1101', 
        #         'sunscreen_1101',
        #         'hammer_1102'
        #     ],
        #     "rotation_euler": [
        #         [-np.pi, 0, np.pi/2],    # Orange
        #         [-np.pi/2, np.pi/4, 0],  # Coke
        #         [np.pi, 0, np.pi/2],     # Realsense Box
        #         [-np.pi, 0, 0],          # Cube
        #         [0, np.pi, 0],           # Coconut Bottle
        #         [np.pi/2, 0, 0],         # Sunscreen
        #         [np.pi/2, 0, 0]          # Hammer
        #     ],
        #     "modeling_file_paths": [
        #         r'data/pointcloud_data/candidiate_objects/orange.npy',
        #         r'data/pointcloud_data/candidiate_objects/coke_can.npy',
        #         r'data/pointcloud_data/candidiate_objects/realsense_box.npy',
        #         r'data/pointcloud_data/candidiate_objects/cube_055.npy',
        #         r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
        #         r'data/pointcloud_data/candidiate_objects/sunscreen.npy',
        #         r'data/pointcloud_data/candidiate_objects/hammer.npy'
        #     ],
        #     "icp_rot_euler": [
        #         False,
        #         False,
        #         True,
        #         True,
        #         True,
        #         True,
        #         # True,
        #         True,
        #     ],
        #     "icp_rot_euler_limits": [
        #         None,
        #         None,
        #         180,
        #         90,
        #         360,
        #         360,
        #         # 360,
        #         360,
        #     ]
        # }
        self.object_configs = self.object_manager.get_object_config(object_identifier)

        # self.object_idx = self.object.index(object_identifier)
        self.object_name = self.object_configs["name"]
        self.euler_xyz = self.object_configs["rotation_euler"]
        self.object_modeling_file_path = self.object_configs["modeling_file_path"]
        self.object_icp_rot_euler = self.object_configs["icp_rot_euler"]
        self.object_icp_rot_euler_limit = self.object_configs["icp_rot_euler_limit"]
        
    def load_sim_trajectory(self):
        # Load and transform simulated trajectory
        sim_traj_file_basename = 'step-0.npy'
        sim_traj_filepath = f'data/trajectory_data/sim_trajectory/{self.object_name}/{sim_traj_file_basename}'
        self.real_traj_adaptor.load_sim_traj_and_transform_hand_to_object(sim_traj_filepath)
        
        
    def locate_object(self, x_keep_range, y_keep_range, z_keep_range, reuse_env_board2cam=True):
        # Locate the object relative to the calibration board
        scene_data_save_dir = f"data/scene_data/{self.object_name}_test_scene_data"
        scene_data_file_name = "test_scene"
        camera_intrinsics_data_dir = f"calibration/calibration_data/{self.camera_name}/camera_intrinsics"
        
        T_calibration_board_to_camera = None
        if reuse_env_board2cam:
            # calibration/calibration_data/camera1/table_to_camera
            camera_extrensics_data_dir = f"calibration/calibration_data/{self.camera_name}/table_to_camera"
            T_calibration_board_to_camera = np.load(f"{camera_extrensics_data_dir}/table_to_camera.npy")
            
        object_pos = self.real_traj_adaptor.locate_object_in_calibration_board_coords(
            scene_data_save_dir=scene_data_save_dir,
            scene_data_file_name=scene_data_file_name,
            camera_intrinsics_data_dir=camera_intrinsics_data_dir,
            calibration_board_info={
                "pattern_size": self.calibration_board_pattern_size,
                "square_size": self.calibration_board_square_size
            },
            overwrite_scene_table_calib_data_if_exists=True,
            vis_scene_point_cloud_in_cam_coord=False,
            vis_scene_point_cloud_in_board_coord=False,
            vis_filtered_point_cloud_in_board_coord=True,
            object_modeling_file_path=self.object_modeling_file_path,
            x_keep_range=x_keep_range,
            y_keep_range=y_keep_range,
            z_keep_range=z_keep_range,
            euler_xyz=self.euler_xyz,
            T_calibration_board_to_camera=T_calibration_board_to_camera,
            locate_rot_by_icp=self.object_icp_rot_euler,
            icp_rot_euler_limit=self.object_icp_rot_euler_limit,
            
        )
        print(f'Object position: {object_pos[:, 3]}')
        
        
    def map_sim_to_real(self):
        # Map simulated trajectory to real world
        self.real_traj_adaptor.map_sim_to_real_handbase_object()

    def compute_real_hand_to_robot_base_transform(self):
        # Compute necessary transformations in the mapped real trajectory
        self.real_traj_adaptor.compute_mapped_real_hand_to_robot_base_transform()
        self.real_traj_adaptor.get_hand_to_robotbase_transform_with_robotbase_reference()


    def save_real_trajectory(self):
        # Save real trajectory data
        save_path = f"data/trajectory_data/real_trajectory/{self.object_name}/step-0.npy"
        self.real_traj_adaptor.save_executable_trajectory(save_path)
        print(f'Real trajectory data saved at: {save_path}')
        
        return save_path
    
    def get_tscaled_robotbase_to_hand_at_first_point(self, t_scale=1):
        # TODO: optimize the scale for each object with specific t_scale
        T_hand_to_base_at_first_point = self.real_traj_adaptor.get_hand_to_robotbase_transform_with_robotbase_reference_with_tscale_at_first_step(t_scale=t_scale)
        self.T_base_to_hand_at_first_point = T_hand_to_base_at_first_point
        return self.T_base_to_hand_at_first_point
        
    def get_finger_angles_at_first_point(self):
        return self.real_traj_adaptor.driven_hand_pos_sim[0]