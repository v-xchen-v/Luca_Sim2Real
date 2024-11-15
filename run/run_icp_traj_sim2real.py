""""""

# Setup module path
import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
# Import necessary packages
from trajectory_processing.trajectory_adaptor import TrajectoryAdaptor

# Initialize TrajectoryAdaptor
real_traj_adaptor = TrajectoryAdaptor()

import numpy as np
orange_upside_rot_eular = [-np.pi, 0, np.pi/2] # base on calibration board base coordinate
# coke_object_rot_eular = [np.pi/2, -np.pi/2, 0] # for coke_1023
# coke_object_rot_eular = [-np.pi/2, np.pi/2-np.pi/4, 0] # for coke_1030
coke_object_rot_eular = [-np.pi/2, np.pi/2, 0] # for coke_1104
cube_rot_euler = [np.pi, 0, np.pi/2-np.pi/4] # 
realsense_box_rot_euler = [np.pi, 0, np.pi/2]
# cube_rot_euler = [-np.pi, 0, 0] # for cube_1023
bottle_coconut_rot_euler = [0, np.pi, 0] # for bottle_coconut_1101
sunscreen_rot_euler = [np.pi/2, 0, 0] # for sunscreen_1101
bleach_cleanser_rot_euler = [0, -np.pi/2, 0] # for bleach_cleanser_1030
hammer_rot_euler = [np.pi/2, 0, 0] # for hammer_1102
duck_rot_euler = [-np.pi/2, np.pi/2, 0] # for duck_1104
tape_measure_rot_euler = [np.pi, np.pi/2, 0] # for tape_measure_1105

# ------------------- Robot table setup ------------------- #
# setup robot and table
CAMERA_NAME = "camera1"
CAPTURE_NEW_TABLE_CALIBRATION_IF_EXISTS = True
# CALIBRATION_BOARD_PATTERN_SIZE = (8, 11)
# CALIBRATION_BOARD_SQUARE_SIZE = 0.02
CALIBRATION_BOARD_PATTERN_SIZE = (5, 8)
CALIBRATION_BOARD_SQUARE_SIZE = 0.03
real_traj_adaptor.calculate_arm_table_robot_transform(
    calibration_data_dir=f"calibration/calibration_data/{CAMERA_NAME}",
    overwrite_if_exists=CAPTURE_NEW_TABLE_CALIBRATION_IF_EXISTS, # Please overwrite if table is moved relative to camera
    calibration_board_info={
        "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
        "square_size": CALIBRATION_BOARD_SQUARE_SIZE
    }
)
# real_traj_adaptor.visualize_arm_table_robot_transform()
# --------------------------------------------------------- #

# ------------------- ICP sim2real ------------------- #
## ----------------------- Sim traj ----------------------- ##
# sim traj config data
sim_traj_object_names = [
    'orange_1024', # worked
    'coke_can_1030',
    'realsense_box_1024',
    'cube_055_1103',
    'bottle_coconut_1101',
    'sunscreen_1101',
    # "bleach_cleanser_1030",
    'hammer_1102',
    'duck_1104',
    'tape_measure_1105',
]

sim_traj_file_basenames = [
    'step-0.npy',
    'step-0.npy',
    'step-0.npy',
    'step-0.npy',
    'step-0.npy',
    'step-0.npy',
    # 'step-0.npy',
    'step-0.npy',
    'step-0.npy',
    'step-5600.npy',
    
]

euler_object_places  = [
    orange_upside_rot_eular,
    coke_object_rot_eular,
    realsense_box_rot_euler,
    cube_rot_euler,
    bottle_coconut_rot_euler,
    sunscreen_rot_euler,
    # bleach_cleanser_rot_euler,
    hammer_rot_euler,
    duck_rot_euler,
    tape_measure_rot_euler,
]
object_modeling_file_paths = [
    r'data/pointcloud_data/candidiate_objects/orange.npy',
    r'data/pointcloud_data/candidiate_objects/coke_can_new.npy',
    r'data/pointcloud_data/candidiate_objects/realsense_box.npy',
    r'data/pointcloud_data/candidiate_objects/cube_055.npy',
    r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
    r'data/pointcloud_data/candidiate_objects/sunscreen.npy',
    # r"data/pointcloud_data/candidiate_objects/bleach_cleanser.npy",
    r'data/pointcloud_data/candidiate_objects/hammer.npy',
    r'data/pointcloud_data/candidiate_objects/duck.npy',
    r'data/pointcloud_data/candidiate_objects/tape_measure.npy',
]

date_tip=1107
scene_data_save_dir = [
    f"data/scene_data/orange_test_scene_data_{date_tip}",
    f"data/scene_data/coke_test_scene_data_{date_tip}",
    f"data/scene_data/realsense_box_test_scene_data_{date_tip}",
    # "data/scene_data/realsense_box_test_scene_data_ver_1024",
    f"data/scene_data/cube_test_scene_data_{date_tip}",
    f"data/scene_data/bottle_coconut_test_scene_data_{date_tip}",
    f"data/scene_data/sunscreen_test_scene_data_{date_tip}",
    # f"data/scene_data/bleach_cleanser_test_scene_data_{date_tip}",
    f'data/scene_data/hammer_test_scene_data_{date_tip}',
    f'data/scene_data/duck_test_scene_data_{date_tip}',
    f'data/scene_data/tape_measure_test_scene_data_{date_tip}',
]

icp_rot_euler = [
    False,
    False,
    True,
    True,
    True,
    True,
    # True,
    True,
    True,
    True,
]

icp_rot_euler_limits = [
    None,
    None,
    180,
    90,
    360,
    360,
    # 360,
    360,
    360,
    180,
]

# The cleanser is hard to do icp since the poor object point cloud quality
object_idx=1
sim_traj_object_name = sim_traj_object_names[object_idx]
sim_traj_file_basename = sim_traj_file_basenames[object_idx]
euler_xyz = euler_object_places[object_idx]
object_modeling_file_path = object_modeling_file_paths[object_idx]
scene_data_save_dir = scene_data_save_dir[object_idx]

print(f"Sim traj object name: {sim_traj_object_name}")
print(f"Sim traj file basename: {sim_traj_file_basename}")
print(f"Object modeling file path: {object_modeling_file_path}")
print(f"Scene data save dir: {scene_data_save_dir}")

# load and vis sim traj
sim_traj_filepath = f'data/trajectory_data/sim_trajectory/{sim_traj_object_name}/{sim_traj_file_basename}'
real_traj_adaptor.load_sim_traj_and_transform_hand_to_object(sim_traj_filepath)
real_traj_adaptor.visualize_sim_world_object_hand_initial_step_transformation()
real_traj_adaptor.animate_sim_hand_approach_object(first_n_steps=200/5)

## -------------------- Locate object -------------------- ##

# scene_data_save_dir = "data/scene_data/cube_test_scene_data"
scene_data_file_name = "test_scene"
camera_intrinsics_data_dir = f"calibration/calibration_data/{CAMERA_NAME}/camera_intrinsics"
# object_modeling_file_path = r'data/pointcloud_data/candidiate_objects/cube_055.npy'
CAPTURE_NEW_SCENE_TABLE_CALIBRATION_IF_EXISTS = True
# # for 11f table
x_keep_range = [-0.30, 0]
y_keep_range = [-0.05, 0.15]
z_keep_range = [None, -0.011]

# for 2f table
# x_keep_range=[-0.35, -0.1]
# y_keep_range=[-0.05, 0.40]
# z_keep_range=[-0.5, 0.072]
# euler_xyz = coke_object_rot_eular

# locate object relative to calibration board
object_pos = real_traj_adaptor.locate_object_in_calibration_board_coords(
    scene_data_save_dir=scene_data_save_dir,
    scene_data_file_name=scene_data_file_name,
    camera_intrinsics_data_dir=camera_intrinsics_data_dir,
    calibration_board_info={
        "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
        "square_size": CALIBRATION_BOARD_SQUARE_SIZE
    },
    overwrite_scene_table_calib_data_if_exists=CAPTURE_NEW_SCENE_TABLE_CALIBRATION_IF_EXISTS,
    vis_scene_point_cloud_in_cam_coord=False,
    vis_scene_point_cloud_in_board_coord=False,
    vis_filtered_point_cloud_in_board_coord=True,
    object_modeling_file_path=object_modeling_file_path,
    x_keep_range=x_keep_range,
    y_keep_range=y_keep_range,
    z_keep_range=z_keep_range,
    T_calibration_board_to_camera=None,
    euler_xyz=euler_xyz,
    locate_rot_by_icp=icp_rot_euler[object_idx],
    icp_rot_euler_limit=icp_rot_euler_limits[object_idx]
)
print(f'object position: {object_pos[:, 3]}')
real_traj_adaptor.visualize_object_in_real()

## -------------------- Mapping Sim2Real with handbase to object transform -------------------- ##
# sim2real
# Step 7: Compute transformation between robot_right_hand_base to robot_base in real world
real_traj_adaptor.map_sim_to_real_handbase_object()
real_traj_adaptor.animate_hand_approach_object_in_real()

## -------------------- Necessary transform computation in mapped real traj -------------------- ##
real_traj_adaptor.compute_mapped_real_hand_to_robot_base_transform()
real_traj_adaptor.animate_hand_to_base_in_real(first_n_steps=200/5)
real_traj_adaptor.get_hand_to_robotbase_transform_with_robotbase_reference()

## --------------------save real traj data-------------------- ##
save_path = f"data/trajectory_data/real_trajectory/{sim_traj_object_name}/step-0.npy"
real_traj_adaptor.save_executable_trajectory(save_path)

# Then you can check the generated real trajectory data in the path: save_path by running the script below:
# - python scripts/real_trajectory/anim_real_traj.py