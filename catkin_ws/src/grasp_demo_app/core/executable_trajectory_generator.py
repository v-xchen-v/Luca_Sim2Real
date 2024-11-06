"""Capture Scene, Determine and Locate Object, Generate Trajectory"""


from .sim2real_trajectory_processor import Sim2RealTrajectoryProcessor
from .object_manager import ObjectManager
from pointcloud_processing.pointcloud_io import load_point_cloud
from pointcloud_processing.icp_matching import get_closest_pcd_match
from pointcloud_processing.pointcloud_io import get_image_and_point_cloud_from_realseanse

class ExecutableTrajectoryGenerator:
    def __init__(self, sim2real_traj_config) -> None:
        # Object Manager
        self.object_manager = ObjectManager()
        
        # Trajectory Processor
        self.processor = Sim2RealTrajectoryProcessor(config=sim2real_traj_config)
        self.processor.setup_robot_table()
        
        # last generated trajectory file path
        self.processor.traj_file_path = None
    
    def capture_scene(self):
        pass
    
    
    def determine_object(self):
        candidate_object_names = ['orange_1024', 'realsense_box_1024']
        
        candidate_object_modeling_files = [self.object_manager.get_object_config(obj)['modeling_file_path']
                                             for obj in candidate_object_names]
        
        candidate_object_pcds = [load_point_cloud(candidate_object_modeling_file) 
                                 for candidate_object_modeling_file in candidate_object_modeling_files]
        
        # TODO: should get point cloud from scene and find best match
        _, best_matching_index, _, _, _, _ = get_closest_pcd_match(target_pcd=None, candidate_pcds=candidate_object_pcds)
        
        # # Dummy logic to determine the object
        # object_idx = 0
        # return object_idx
        return candidate_object_names[best_matching_index]
    
    
    # def locate_object(self):
    #     pass
    
    
    def generate_trajectory(self):
        # Determine the object
        object_name = self.determine_object()
        self.processor.configure_object_settings(object_identifier=object_name)
        
        
        # for 2f table
        # TODO: move it to config
        x_keep_range=[-0.45, -0.1]
        y_keep_range=[-0.05, 0.40]
        z_keep_range=[-0.5, 0.070]
        
        # Trajectory generation
        self.processor.load_sim_trajectory()
        self.processor.locate_object(x_keep_range=x_keep_range, 
                                     y_keep_range=y_keep_range, 
                                     z_keep_range=z_keep_range)
        self.processor.map_sim_to_real()
        self.processor.compute_real_hand_to_robot_base_transform()
        self.traj_file_path = self.processor.save_real_trajectory()
        
        