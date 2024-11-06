"""Capture Scene, Determine and Locate Object, Generate Trajectory"""


from .sim2real_trajectory_processor import Sim2RealTrajectoryProcessor

class ExecutableTrajectoryGenerator:
    def __init__(self, sim2real_traj_config) -> None:
        self.processor = Sim2RealTrajectoryProcessor(config=sim2real_traj_config)
        self.processor.setup_robot_table()
        
        # last generated trajectory file path
        self.processor.traj_file_path = None
    
    def capture_scene(self):
        pass
    
    
    def determine_object(self):
        # Dummy logic to determine the object
        object_idx = 0
        return object_idx
    
    
    # def locate_object(self):
    #     pass
    
    
    def generate_trajectory(self):
        # Determine the object
        object_idx = self.determine_object()
        self.processor.configure_object_settings(object_idx=object_idx)
        
        
        # Trajectory generation
        self.processor.load_sim_trajectory()
        self.processor.locate_object(x_keep_range=[-0.30, 0], y_keep_range=[-0.05, 0.15], z_keep_range=[None, -0.011])
        self.processor.map_sim_to_real()
        self.processor.compute_real_hand_to_robot_base_transform()
        self.traj_file_path = self.processor.save_real_trajectory()
        
        