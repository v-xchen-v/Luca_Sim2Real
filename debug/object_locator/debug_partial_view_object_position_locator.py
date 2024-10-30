import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    

from pointcloud_processing.object_locator import ObjectPositionLocator

CAMERA_NAME = "camera1"
CAPTURE_NEW_TABLE_CALIBRATION = False
CALIBRATION_BOARD_PATTERN_SIZE = (8, 11)
CALIBRATION_BOARD_SQUARE_SIZE = 0.02
# CALIBRATION_BOARD_PATTERN_SIZE = (5, 8)
# CALIBRATION_BOARD_SQUARE_SIZE = 0.03

object_locator = ObjectPositionLocator(
    scene_data_save_dir="data/scene_data/test_scene_data",
    scene_data_file_name="test_scene",
    camera_intrinsics_data_dir=f"calibration/calibration_data/{CAMERA_NAME}/camera_intrinsics",
    calibration_board_info={
        "pattern_size": CALIBRATION_BOARD_PATTERN_SIZE,
        "square_size": CALIBRATION_BOARD_SQUARE_SIZE
    },
    report_dir="data/scene_data/test_scene_data",
    overwrite_if_exists=CAPTURE_NEW_TABLE_CALIBRATION,
    vis_scene_point_cloud_in_cam_coord=True,
    vis_scene_point_cloud_in_board_coord=True,
    vis_filtered_point_cloud_in_board_coord=True,
    )

object_center = object_locator.locate_partial_view_object_position(
    x_range=[-0.13, 0],
    y_range=[-0.05, 0.10],
    z_range=[None, -0.02]
    # x_range=[None, None],
    # y_range=[None, None],
    # z_range=[None, 0]
)
print(f"Object center position: {object_center}")