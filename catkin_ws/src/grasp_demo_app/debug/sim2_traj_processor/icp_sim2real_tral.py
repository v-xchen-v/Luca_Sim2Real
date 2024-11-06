import os, sys
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

from core.sim2real_trajectory_processor import Sim2RealTrajectoryProcessor

# Example usage
processor = Sim2RealTrajectoryProcessor()
processor.setup_robot_table()
processor.configure_object_settings(object_idx=3)  # Example: Cube configuration
processor.load_sim_trajectory()
processor.locate_object(x_keep_range=[-0.30, 0], y_keep_range=[-0.05, 0.15], z_keep_range=[None, -0.011])
processor.map_sim_to_real()
processor.compute_real_hand_to_robot_base_transform()
processor.save_real_trajectory()