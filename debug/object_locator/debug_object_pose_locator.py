import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from pointcloud_processing.object_locator import ObjectPoseLocator
from scipy.spatial.transform import Rotation as R
import numpy as np

CAMERA_NAME = "camera1"
CAPTURE_NEW_TABLE_CALIBRATION = False
# CALIBRATION_BOARD_PATTERN_SIZE = (8, 11)
# CALIBRATION_BOARD_SQUARE_SIZE = 0.02
CALIBRATION_BOARD_PATTERN_SIZE = (5, 8)
CALIBRATION_BOARD_SQUARE_SIZE = 0.03

# Ref: data/object_models/cube/cube_sim_axes.jpg
# aka, board_to_modeling
R_cube_modeling_in_placed_in_board_coord = R.from_euler('XYZ', [0, 0, -np.pi/2]).as_matrix()

object_locator = ObjectPoseLocator(
        scene_data_save_dir="data/scene_data/cube_test_scene_data",
        scene_data_file_name="test_scene",
        camera_intrinsics_data_dir=f"calibration/calibration_data/{CAMERA_NAME}/camera_intrinsics",
        calibration_board_info={
            "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
            "square_size": CALIBRATION_BOARD_SQUARE_SIZE
        },
        report_dir="data/scene_data/cube_test_scene_data",
        overwrite_if_exists=CAPTURE_NEW_TABLE_CALIBRATION,
        vis_scene_point_cloud_in_cam_coord=False,
        vis_scene_point_cloud_in_board_coord=False,
        vis_filtered_point_cloud_in_board_coord=True,
        object_modeling_file_path=r'data/pointcloud_data/candidiate_objects/cube_055.npy',
        R_calibration_board_to_object_placed_face_robot=R_cube_modeling_in_placed_in_board_coord
    )

object_center = object_locator.locate_partial_view_object_position(
    x_range=[-0.20, 0],
    y_range=[-0.05, 0.10],
    z_range=[None, -0.02]
    # x_range=[None, None],
    # y_range=[None, None],
    # z_range=[None, 0]
)
print(f"Object center position: {object_center}")
fine_object_center = object_locator.locate_object_position()
print(f'Fine object center position: {fine_object_center}')

T_board_to_object = object_locator.locate_object_pose()
print(f'Object pose: {T_board_to_object}')
print(f'Object position: {T_board_to_object[:3, 3]}')
print(f'Object orientation: {R.from_matrix(T_board_to_object[:3, :3]).as_euler("XYZ")*180/np.pi}')