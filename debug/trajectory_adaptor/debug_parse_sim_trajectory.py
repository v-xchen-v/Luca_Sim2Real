"""
Check parsed sim_trajectory data and live visualize the transfromation between the object and right hand base
"""
import numpy as np

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor

# Initialize the trajectory adaptor with pre-computed calibration data
adaptor = TrajectoryAdaptor()

# Extract the sim_trajectory data
(T_object_to_right_hand_base, 
 driven_hand_joint_pos, 
 right_hand_base_pos, 
 grasp_flag) = adaptor.parse_sim_trajectory(r'data/trajectory_data/sim_trajectory/coka_can_1017/step-0.npy')

# Visualize the first step, relative transformation between the object and right hand base
adaptor.frame_manager.visualize_transforation("object_sim", "right_hand_base_sim")
