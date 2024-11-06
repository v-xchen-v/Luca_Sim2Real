"""Capture Scene, Determine and Locate Object, Generate Trajectory"""


from .sim2real_trajectory_processor import Sim2RealTrajectoryProcessor

class ExecutableTrajectoryGenerator:
    def __init__(self) -> None:
        self.processor = Sim2RealTrajectoryProcessor()
        self.processor.setup_robot_table()
        self.processor.configure_object_settings(object_idx=3)
    
    
    def capture_scene(self):
        pass
    
    
    # def determine_object(self):
    #     pass
    
    
    # def locate_object(self):
    #     pass
    
    
    def generate_trajectory(self):
        self.processor.load_sim_trajectory()
        self.processor.locate_object(x_keep_range=[-0.30, 0], y_keep_range=[-0.05, 0.15], z_keep_range=[None, -0.011])
        self.processor.map_sim_to_real()
        self.processor.compute_real_hand_to_robot_base_transform()
        self.processor.save_real_trajectory()