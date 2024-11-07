"""Capture Scene, Determine and Locate Object, Generate Trajectory"""


from .sim2real_trajectory_processor import Sim2RealTrajectoryProcessor
from .object_manager import ObjectManager
from pointcloud_processing.pointcloud_io import load_npy_file_as_point_cloud
from pointcloud_processing.icp_matching import get_closest_pcd_match
from pointcloud_processing.pointcloud_io import get_image_and_point_cloud_from_realseanse
from pointcloud_processing.object_point_cloud_extractor import ObjectPointCloudExtractor
import open3d as o3d
import json


class ExecutableTrajectoryGenerator:
    def __init__(self, sim2real_traj_config) -> None:
        # Object Manager
        self.object_manager = ObjectManager()
        
        self._load_config(sim2real_traj_config)
        
        # Trajectory Processor
        self.processor = Sim2RealTrajectoryProcessor(config=sim2real_traj_config)
        self.processor.setup_robot_table()
        
        # last generated trajectory file path
        self.processor.traj_file_path = None
        
        # for 2f table
        # TODO: move it to config
        self.x_keep_range=[-0.40, -0.1] # x can
        self.y_keep_range=[-0.05, 0.40]
        self.z_keep_range=[-0.5, 0.068 -0.073]
        # self.z_keep_range=[-0.5, -0.004]
        
        # TODO: restructure code to avoid computing this multiple times
        self.object_pc_extractor = ObjectPointCloudExtractor(
            T_calibration_board_to_camera=self.processor.real_traj_adaptor.frame_manager.get_transformation("calibration_board_real", "camera_real"))
    
    def _load_config(self, config):
        # Load configuration from a dictionary or a json file
        if isinstance(config, str):
            with open(config, 'r') as f:
                self.config = json.load(f)
        elif isinstance(config, dict):
            self.config = config
        else:
            raise ValueError("Invalid config type. Use a dictionary or a json file.")
        
    def capture_scene(self):
        pass
    
    
    def determine_object(self):
        # candidate_object_names = ['orange_1024', 'realsense_box_1024']
        candidate_object_names = self.config["candidiates"]
        
        # if only one object, return it
        if len(candidate_object_names) == 1:
            return candidate_object_names[0]
        
        candidate_object_modeling_files = [self.object_manager.get_object_config(obj)['modeling_file_path']
                                             for obj in candidate_object_names]
        
        candidate_object_pcds = [load_npy_file_as_point_cloud(candidate_object_modeling_file) 
                                 for candidate_object_modeling_file in candidate_object_modeling_files]
        
        scene_pcd, scene_color_image = get_image_and_point_cloud_from_realseanse()
        object_pcd_in_board_coord, _, _ = self.object_pc_extractor.extract(scene_pcd, 
                                                                        x_keep_range=self.x_keep_range,
                                                                        y_keep_range=self.y_keep_range, 
                                                                        z_keep_range=self.z_keep_range)
        object_roi_color_image = self.object_pc_extractor.get_object_rgb_in_cam_coord(scene_color_image)
        if False:
            vis_pcds = []
            for item in candidate_object_pcds:
                vis_pcds.append(item)
            vis_pcds.append(object_pcd_in_board_coord)
            o3d.visualization.draw_geometries(vis_pcds)
    

        # TODO: should get point cloud from scene and find best match
        _, best_matching_index, _, _, _, _ = get_closest_pcd_match(target_pcd=object_pcd_in_board_coord, 
                                                                   candidate_pcds=candidate_object_pcds)
        
        # # Dummy logic to determine the object
        # object_idx = 0
        # return object_idx
        print(f"best matching object: {candidate_object_names[best_matching_index]}")
        return candidate_object_names[best_matching_index]
    
    
    # def locate_object(self):
    #     pass
    
    
    def generate_trajectory(self):
        # Determine the object
        object_name = self.determine_object()
        self.processor.configure_object_settings(object_identifier=object_name)
        
        

        # Trajectory generation
        self.processor.load_sim_trajectory()
        self.processor.locate_object(x_keep_range=self.x_keep_range, 
                                     y_keep_range=self.y_keep_range, 
                                     z_keep_range=self.z_keep_range)
        self.processor.map_sim_to_real()
        self.processor.compute_real_hand_to_robot_base_transform()
        self.traj_file_path = self.processor.save_real_trajectory()
        
        