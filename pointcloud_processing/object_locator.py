"""Given an image and a point cloud of scene, this ObjectLocator will computed the object center position(xyz), and 
rotation from modeling orientation."""

import numpy as np
import cv2
import open3d as o3d
from pointcloud_processing.pointcloud_io import save_image_and_point_cloud_from_realsense, load_point_cloud
from calibration.calibration_precomputed_data_loader import load_camera_intrinsics_from_npy
from calibration.calibrate_board_to_camera import compute_table_to_camera
from pointcloud_processing.table_filter import filter_point_outside_operation_area
from pointcloud_processing.transformations import transform_point_cloud_from_camera_to_table
from pytransform3d.transformations import invert_transform

class ObjectLocatorBase:
    """Shared methods and attributes for ObjectPositionLocator and ObjectPoseLocator."""
    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 calibration_board_info: dict,
                 report_dir: str,
                 overwrite_if_exists: bool,
                 ) -> None:
        # store input arguments
        self.scene_data_save_dir = scene_data_save_dir
        self.scene_data_file_name = scene_data_file_name
        self.camera_intrinsics_data_dir = camera_intrinsics_data_dir
        self.overwrite_if_exists = overwrite_if_exists,
        self.calibration_board_info = calibration_board_info
        self.report_dir = report_dir
        
        # store intermediate data
        self.scene_image = None # BGR image
        self.scene_point_cloud_in_camera_coord = None
        self.T_calibration_board_to_camera = None
        self.filtered_scene_point_cloud = None
        
    def _capture_scene(self):
        save_image_and_point_cloud_from_realsense(
            self.scene_data_save_dir, 
            self.scene_data_file_name, 
            self.overwrite_if_exists)
        
    def _load_scene(self):
        self.scene_image = cv2.imread(f"{self.scene_data_save_dir}/{self.scene_data_file_name}.png")

        self.scene_point_cloud_in_camera_coord = load_point_cloud(
            f"{self.scene_data_save_dir}/{self.scene_data_file_name}.pcd")
    
    def _process_scene_image(self):
        """Find calibration board and do table """
        if self.scene_image is None:
            raise ValueError("No scene image is loaded.")
        
        mtx, dist = load_camera_intrinsics_from_npy(self.camera_intrinsics_data_dir)
        self.T_calibration_board_to_camera = \
            compute_table_to_camera(image=self.scene_image,
                                    pattern_size=self.calibration_board_info['pattern_size'],
                                    square_size=self.calibration_board_info['square_size'],
                                    mtx=mtx,
                                    dist=dist,
                                    report_dir=self.report_dir,
                                    error_threshold=0.15)
    
class ObjectPositionLocator(ObjectLocatorBase):
    """
    Given a image and point cloud of scene include a table, a calibration board attach on table and a object on table,
    the ObjectPositionLocator will compute the object center position(xyz).
    
    Some object is same for grasp after rotation, so that only need locating position, not rotation.
    """
    
    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 calibration_board_info: dict = {
                     "pattern_size": (5, 8),
                     "square_size": 0.03
                    },
                 report_dir: str = None,
                 overwrite_if_exists: bool = False,
                 vis_filtered_point_cloud: bool = False,
                 vis_scene_point_cloud: bool = False
                 ) -> None:
        super().__init__(scene_data_save_dir, 
                         scene_data_file_name, 
                         camera_intrinsics_data_dir, 
                         calibration_board_info, 
                         report_dir, 
                         overwrite_if_exists)
        # intermediate data
        self.scene_point_cloud_in_board_coord = None
        
        # configure visualization
        self.vis_scene_point_cloud = vis_scene_point_cloud
        self.vis_filtered_point_cloud = vis_filtered_point_cloud

    def locate_object_position(self, 
                               x_range=[None, None], 
                               y_range=[None, None], 
                               z_range=[None, None]):
        self._capture_scene()
        self._load_scene()
        self._process_scene_image()
        self._process_scene_pointcloud(x_range, y_range, z_range) # [num_points, 3]
        if self.filtered_scene_point_cloud_in_board_coord is None:
            raise ValueError("No object point cloud is found after filtering.")
        
        # compute the object center position
        object_center = np.mean(self.filtered_scene_point_cloud_in_board_coord, axis=0)
        return object_center  
        
    def _process_scene_pointcloud(self, x_range, y_range, z_range):
        self.scene_point_cloud_in_board_coord = \
            transform_point_cloud_from_camera_to_table(self.scene_point_cloud_in_camera_coord, 
                                                       T_camera_to_table=invert_transform(self.T_calibration_board_to_camera))
        
        if self.vis_scene_point_cloud:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.scene_point_cloud_in_board_coord)
            o3d.visualization.draw_geometries([pcd])
            
        self.filtered_scene_point_cloud_in_board_coord = \
            filter_point_outside_operation_area(
                self.scene_point_cloud_in_board_coord,
                x_range=x_range,
                y_range=y_range,
                z_range=z_range)
        
        if self.vis_filtered_point_cloud:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.filtered_scene_point_cloud_in_board_coord)
            o3d.visualization.draw_geometries([pcd])
        
class ObjectPoseLocator(ObjectLocatorBase):
    """
    Given a image and point cloud of scene include a table, a calibration board attach on table and a object on table,
    the ObjectPositionLocator will compute the object center position(xyz) and the rotation(xyzw) of object from 
    modeling orientation.
    """

    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 object_modeling_file_path:str,
                 overwrite_if_exists: bool = False) -> None:
        super().__init__(scene_data_save_dir, scene_data_file_name, camera_intrinsics_data_dir, overwrite_if_exists)

    
    def _load_object_model(self):
        pass
    
    def _process_scene_pointcloud(self):
        pass
    
    def locate_object_pose(self):
        pass