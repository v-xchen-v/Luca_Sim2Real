import numpy as np
from pointcloud_processing.transformations import transform_point_cloud_from_camera_to_table, invert_transform
from pointcloud_processing.table_filter import filter_point_outside_operation_area
from pointcloud_processing.pointcloud_io import load_npy_as_point_cloud

class ObjectPointCloudExtractor:
    def __init__(self, T_calibration_board_to_camera):
        self.T_calibration_board_to_camera = T_calibration_board_to_camera
                
    def extract(self, scene_pcd, x_keep_range, y_keep_range, z_keep_range):
        scene_pcd_in_board_coord = \
            transform_point_cloud_from_camera_to_table(scene_pcd, 
                                                    T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                    T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        
        points_filtered = filter_point_outside_operation_area(scene_pcd_in_board_coord, 
                                                            x_range=x_keep_range, y_range=y_keep_range, z_range=z_keep_range)
        
        object_pcd_in_board_coord = load_npy_as_point_cloud(points_filtered)
        return object_pcd_in_board_coord, scene_pcd_in_board_coord
