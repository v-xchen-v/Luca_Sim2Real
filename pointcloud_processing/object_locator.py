"""Given an image and a point cloud of scene, this ObjectLocator will computed the object center position(xyz), and 
rotation from modeling orientation."""

import numpy as np
import cv2
import open3d as o3d
class ObjectLocatorBase:
    """Shared methods and attributes for ObjectPositionLocator and ObjectPoseLocator."""
    
    def _capture_scene(self):
        pass

    def _load_scene(self):
        pass
    
    def _process_scene_image(self):
        pass
    
    
class ObjectPositionLocator(ObjectLocatorBase):
    """
    Given a image and point cloud of scene include a table, a calibration board attach on table and a object on table,
    the ObjectPositionLocator will compute the object center position(xyz).
    """
    def __init__(self):
        pass
    

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
    def __init__(self) -> None:
        pass
    
    def _load_object_model(self):
        pass
    
    def _process_scene_pointcloud(self):
        pass
    
    def locate_object_pose(self):
        pass