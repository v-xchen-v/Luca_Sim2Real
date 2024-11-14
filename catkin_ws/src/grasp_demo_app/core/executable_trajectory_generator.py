"""Capture Scene, Determine and Locate Object, Generate Trajectory"""


from .sim2real_trajectory_processor import Sim2RealTrajectoryProcessor
from .object_manager import ObjectManager
from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud
from pointcloud_processing.icp_matching import get_closest_pcd_match
from pointcloud_processing.pointcloud_io import get_image_and_point_cloud_from_realseanse
from pointcloud_processing.object_point_cloud_extractor import ObjectPointCloudExtractor
import open3d as o3d
import json
import cv2
from .object_classifier import get_object_name_from_clip
from pointcloud_processing.pointcloud_exceptions import ICPFitnessException
import select, sys
from rembg import remove
import time
import numpy as np

class ExecutableTrajectoryGenerator:
    def __init__(self, sim2real_traj_config) -> None:
        # Object Manager
        self.object_manager = ObjectManager()
        
        self._load_config(sim2real_traj_config)
        # Trajectory Processor
        self.processor = Sim2RealTrajectoryProcessor(config=sim2real_traj_config)
        
        
        # last generated trajectory file path
        self.processor.traj_file_path = None
        
        # for 2f table
        # # TODO: move it to config
        # self.x_keep_range=[-0.40, -0.1] # x can
        # self.y_keep_range=[-0.05, 0.40]
        # self.z_keep_range=[-0.5, 0.068 -0.073]
        # self.z_keep_range=[-0.5, -0.004]
        
        # self.x_keep_range = [-0.2, +0.2]
        # self.y_keep_range = [-0.15, +0.15]
        # self.z_keep_range = [-0.3, -0.004]
        
        self.classifier_object_in_loop = self.config["classifier_object_in_loop"]
        
        self.x_keep_range = self.config["point_cloud_x_keep_range"]
        self.y_keep_range = self.config["point_cloud_y_keep_range"]
        self.z_keep_range = self.config["point_cloud_z_keep_range"]
        
        self.vis_object_pcd = self.config["vis_object_pcd"]
        self.vis_object_icp = self.config["vis_object_icp"]
        self.roi_object_remove_bg = self.config["roi_object_remove_bg"]
        self.roi_object_scale_factor = self.config["roi_object_scale_factor"]
        self.object_roi_x_min_range = self.config["object_roi_x_min_range"]
        self.object_roi_y_min_range = self.config["object_roi_y_min_range"]
        
        self.calibration_error_threshold = self.config["calibration_error_threshold"]        
        # object management configs
        self.object_manager_configs = None
        
    
    def _load_config(self, config):
        # Load configuration from a dictionary or a json file
        if isinstance(config, str):
            with open(config, 'r') as f:
                self.config = json.load(f)
        elif isinstance(config, dict):
            self.config = config
        else:
            raise ValueError("Invalid config type. Use a dictionary or a json file.")
        
    def initialize(self):
        self.processor.setup_robot_table(calibration_error_threshold=self.calibration_error_threshold)
        
        # TODO: restructure code to avoid computing this multiple times
        self.object_pc_extractor = ObjectPointCloudExtractor(
            T_calibration_board_to_camera=self.processor.real_traj_adaptor.frame_manager.get_transformation("calibration_board_real", "camera_real"))
    
    def _crop_object_roi(self, candidate_object_names, use_pcd, scale_factor,
                         x_min_range, y_min_range):
        if use_pcd:
            candidate_object_modeling_files = [self.object_manager.get_object_config(obj)['modeling_file_path']
                                                for obj in candidate_object_names]
            
            candidate_object_pcds = [load_npy_file_as_point_cloud(candidate_object_modeling_file) 
                                    for candidate_object_modeling_file in candidate_object_modeling_files]
            
            scene_pcd, scene_color_image = get_image_and_point_cloud_from_realseanse()
            object_pcd_in_board_coord, _, _ = self.object_pc_extractor.extract(scene_pcd, 
                                                                            x_keep_range=self.x_keep_range,
                                                                            y_keep_range=self.y_keep_range, 
                                                                            z_keep_range=self.z_keep_range)
            if False:
                vis_pcds = []
                for item in candidate_object_pcds:
                    vis_pcds.append(item)
                vis_pcds.append(object_pcd_in_board_coord)
                o3d.visualization.draw_geometries(vis_pcds)
                
            object_roi_color_image = \
                self.object_pc_extractor.get_object_rgb_in_cam_coord(
                    scene_color_image, 
                    scale_factor=scale_factor,
                    x_min_range=x_min_range,
                    y_min_range=y_min_range)
                
            if object_roi_color_image is None: 
                raise ValueError("object_roi_color_image is None")
            
            return object_roi_color_image   
        else:
            _, scene_color_image = get_image_and_point_cloud_from_realseanse()
            return scene_color_image[int(x_min_range[0]):int(x_min_range[1]), 
                                     int(y_min_range[0]):int(y_min_range[1])]
            
    def determine_object(self, use_pcd=False):
        use_remove_bg = self.roi_object_remove_bg
        
        # candidate_object_names = ['orange_1024', 'realsense_box_1024']
        candidate_object_names = self.config["candidiates"]
        
        # if only one object, return it
        if len(candidate_object_names) == 1:
            self.object_manager_configs = self.object_manager.get_object_config(candidate_object_names[0])
            return candidate_object_names[0]
        
        object_roi_color_image_path = 'data/debug_data/pointcloud_data/camera_captures/object_roi_color_image.png'
        
        if not self.classifier_object_in_loop:
            object_roi_color_image = self._crop_object_roi(
                candidate_object_names, 
                use_pcd=use_pcd, scale_factor=self.roi_object_scale_factor,
                x_min_range=self.object_roi_x_min_range,
                y_min_range=self.object_roi_y_min_range)
            if use_remove_bg:
                object_roi_color_image = remove(object_roi_color_image)
            cv2.imwrite(object_roi_color_image_path, object_roi_color_image)
            # Add a short delay to ensure file system writes complete
            time.sleep(0.3)
            candidate_object_name = get_object_name_from_clip(object_roi_color_image_path)
        else:
            # loop until success which when human key press any key
            while True:
                object_roi_color_image = self._crop_object_roi(
                    candidate_object_names, 
                    use_pcd=use_pcd, scale_factor=self.roi_object_scale_factor,
                    x_min_range=self.object_roi_x_min_range,
                    y_min_range=self.object_roi_y_min_range)
                if use_remove_bg:
                    object_roi_color_image = remove(object_roi_color_image)
                    alpha_channel = object_roi_color_image[:, :, 3]
                    coord = np.argwhere(alpha_channel > 200)
                    if coord.any():
                        x_min, y_min = coord.min(axis=0)
                        x_max, y_max = coord.max(axis=0)
                        y_min = max(0, y_min)
                        x_max = min(object_roi_color_image.shape[0], x_max)

                        object_roi_color_image = object_roi_color_image[x_min:x_max, y_min:y_max, :3]
                        if np.any(np.array(object_roi_color_image.shape) == 0):
                            continue
                            
                cv2.imwrite(object_roi_color_image_path, object_roi_color_image)
                
                # Add a short delay to ensure file system writes complete
                time.sleep(0.3)
                candidate_object_name = get_object_name_from_clip(object_roi_color_image_path)
                print(f"------ candidate_object_name: {candidate_object_name} ------")
                print("Press any key if the object is correctly identified")

                # Waiting for Enter key input from the user to break the loop
                # The select function waits for input, with a timeout of 1 second
                if select.select([sys.stdin], [], [], 1)[0]:
                    # If input is detected, read it and break the loop
                    print("Exiting the loop.")
                    break
                else:
                    # Continue the loop if no input is given within 1 second
                    print("No input detected, continuing...")

        
        self.object_manager_configs = self.object_manager.get_object_config(candidate_object_name)
        # # Dummy logic to determine the object
        # object_idx = 0
        # return object_idx
        # print(f"best matching object: {candidate_object_names[best_matching_index]}")
        # return candidate_object_names[best_matching_index]
        print(f"best matching object: {candidate_object_name}")
        return candidate_object_name
    
    
    # def locate_object(self):
    #     pass
    
    
    def generate_trajectory(self, vis_sim_initial_setup, anim_sim_hand_approach,
                            vis_object_in_real, anim_real_hand_approach_object):
        # Determine the object
        object_name = self.determine_object()
        self.processor.configure_object_settings(object_identifier=object_name)
        
        

        # Trajectory generation
        self.processor.load_sim_trajectory(vis_sim_initial_setup=vis_sim_initial_setup, 
                                           anim_sim_hand_approach=anim_sim_hand_approach)
        
        try:
            self.processor.locate_object(x_keep_range=self.x_keep_range, 
                                        y_keep_range=self.y_keep_range, 
                                        z_keep_range=self.z_keep_range,
                                        vis_object_in_real=vis_object_in_real,
                                        vis_object_point_cloud=self.vis_object_pcd,
                                        vis_object_icp=self.vis_object_icp)
        except ICPFitnessException as e:
            raise ICPFitnessException(f"ICP Fitness Exception: {e}")
            
        self.processor.map_sim_to_real(anim_real_hand_approach_object=anim_real_hand_approach_object)
            
        self.processor.compute_real_hand_to_robot_base_transform()
        self.traj_file_path = self.processor.save_real_trajectory(overwrite_if_exists=True)
        
        