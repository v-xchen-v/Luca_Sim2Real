"""Given an image and a point cloud of scene, this ObjectLocator will computed the object center position(xyz), and 
rotation from modeling orientation."""

import numpy as np
import cv2
import open3d as o3d
from .pointcloud_io import save_image_and_point_cloud_from_realsense, load_point_cloud
from calibration.calibration_data_utils import get_calibration_data
class ObjectLocatorBase:
    """Shared methods and attributes for ObjectPositionLocator and ObjectPoseLocator."""
    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 overwrite_if_exists: bool =False,
                 ) -> None:
        # store input arguments
        self.scene_data_save_dir = scene_data_save_dir
        self.scene_data_file_name = scene_data_file_name
        self.camera_intrinsics_data_dir = camera_intrinsics_data_dir
        self.overwrite_if_exists = overwrite_if_exists
        
        
        self.scene_image = None
        self.scene_point_cloud = None
        
    def _capture_scene(self):
        save_image_and_point_cloud_from_realsense(
            self.scene_data_save_dir, 
            self.scene_data_file_name, 
            self.overwrite_if_exists)

    def _load_scene(self):
        self.scene_image = cv2.imread(
            f"{self.scene_data_save_dir}/{self.scene_data_file_name}.png")
        self.scene_point_cloud = load_point_cloud(
            f"{self.scene_data_save_dir}/{self.scene_data_file_name}.pcd")
    
    def _process_scene_image(self):
        """Find calibration board and do table """
        if self.scene_image is None:
            raise ValueError("No scene image is loaded.")
        
        pass
    
    
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
                 overwrite_if_exists: bool = False,
                 ) -> None:
        super().__init__(scene_data_save_dir, scene_data_file_name, camera_intrinsics_data_dir, overwrite_if_exists)

    def locate_object_position(self):
        pass
    
    def _process_scene_pointcloud(self):
        pass
    
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