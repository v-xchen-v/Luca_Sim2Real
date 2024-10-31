""""""


import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor
real_traj_adaptor = TrajectoryAdaptor()

# sim traj config data
sim_traj_object_name = "orange_1024"
sim_traj_file_basename = "step-0.npy"

# load and vis sim traj
sim_traj_filepath = f'data/trajectory_data/sim_trajectory/{sim_traj_object_name}/{sim_traj_file_basename}'
real_traj_adaptor.load_sim_traj_and_transform_hand_to_object(sim_traj_filepath)
# real_traj_adaptor.visualize_sim_world_object_hand_initial_step_transformation()
# real_traj_adaptor.animate_sim_hand_approach_object(first_n_steps=200/5)


# setup robot and table
CAMERA_NAME = "camera1"
CAPTURE_NEW_TABLE_CALIBRATION_IF_EXISTS = False
CALIBRATION_BOARD_PATTERN_SIZE = (8, 11)
CALIBRATION_BOARD_SQUARE_SIZE = 0.02
# CALIBRATION_BOARD_PATTERN_SIZE = (5, 8)
# CALIBRATION_BOARD_SQUARE_SIZE = 0.03
real_traj_adaptor.calculate_arm_table_robot_transform(
    calibration_data_dir=f"calibration/calibration_data/{CAMERA_NAME}",
    overwrite_if_exists=CAPTURE_NEW_TABLE_CALIBRATION_IF_EXISTS, # Please overwrite if table is moved relative to camera
    calibration_board_info={
        "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
        "square_size": CALIBRATION_BOARD_SQUARE_SIZE
    }
)
real_traj_adaptor.visualize_arm_table_robot_transform()

# locate object


# sim2real

# save real traj data