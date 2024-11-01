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
import os
from pointcloud_processing.icp_matching import align_source_to_target

class ObjectLocatorBase:
    """Shared methods and attributes for ObjectPositionLocator and ObjectPoseLocator."""
    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 calibration_board_info: dict,
                 report_dir: str,
                 overwrite_if_exists: bool,
                 T_calibration_board_to_camera: np.ndarray = None,
                 ) -> None:
        # store input arguments
        self.scene_data_save_dir = scene_data_save_dir
        self.scene_data_file_name = scene_data_file_name
        self.camera_intrinsics_data_dir = camera_intrinsics_data_dir
        self.overwrite_if_exists = overwrite_if_exists
        self.calibration_board_info = calibration_board_info
        self.report_dir = report_dir
        self.T_calibration_board_to_camera = T_calibration_board_to_camera
        
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
    
    def _get_transform_table_to_camera(self, T_calibration_board_to_camera: np.ndarray) -> np.ndarray:
        if T_calibration_board_to_camera is None:
            self.T_calibration_board_to_camera = self._process_scene_image()
        else:
            self.T_calibration_board_to_camera = T_calibration_board_to_camera
        
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
        return self.T_calibration_board_to_camera
    
class ObjectPositionLocator(ObjectLocatorBase):
    """
    Given a image and point cloud of scene include a table, a calibration board attach on table and a object on table,
    the ObjectPositionLocator will compute the object center position(xyz).
    
    Some object is same for grasp after rotation, so that only need locating position, not rotation.
    
    Table to camera calibration could do each time or used previous setup value at very initial time. Once the table is 
    moved relative to camera, the calibration should be done again.
    """
    
    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 calibration_board_info: dict = {
                     "pattern_size": (5, 8),
                     "square_size": 0.03
                    },
                 object_modeling_file_path:str = None,
                 report_dir: str = None,
                 overwrite_if_exists: bool = False,
                 vis_filtered_point_cloud_in_board_coord: bool = False,
                 vis_scene_point_cloud_in_cam_coord: bool = False,
                 vis_scene_point_cloud_in_board_coord: bool = False,
                 vis_filtered_point_cloud_in_cam_coord: bool = False,
                 T_calibration_board_to_camera: np.ndarray = None
                 ) -> None:
        super().__init__(scene_data_save_dir, 
                         scene_data_file_name, 
                         camera_intrinsics_data_dir, 
                         calibration_board_info, 
                         report_dir, 
                         overwrite_if_exists,
                         T_calibration_board_to_camera)
        # custom input arguments
        self.object_modeling_file_path = object_modeling_file_path
        
        # intermediate data
        self.scene_point_cloud_in_board_coord = None
        
        # configure visualization
        self.vis_scene_point_cloud_in_camera_coord = vis_scene_point_cloud_in_cam_coord
        self.vis_scene_point_cloud_in_board_coord = vis_scene_point_cloud_in_board_coord
        self.vis_filtered_point_cloud_in_board_coord = vis_filtered_point_cloud_in_board_coord
        self.vis_filtered_point_cloud_in_cam_coord = vis_filtered_point_cloud_in_cam_coord

    def locate_partial_view_object_position(self, 
                               x_range=[None, None], 
                               y_range=[None, None], 
                               z_range=[None, None],
                               save_filtered_point_cloud: bool = True) -> np.ndarray:
        self._capture_scene()
        self._load_scene()
        self._get_transform_table_to_camera(self.T_calibration_board_to_camera)
        self._process_scene_pointcloud(x_range, y_range, z_range) # [num_points, 3]
        if self.filtered_scene_point_cloud_in_board_coord is None:
            raise ValueError("No object point cloud is found after filtering.")
        
        if save_filtered_point_cloud:
            if len(self.filtered_scene_point_cloud_in_board_coord) == 0:
                raise ValueError("No points to save.")
            
            # create directory if not exists
            np.save(f"{self.scene_data_save_dir}/{self.scene_data_file_name}_filtered_point_cloud.npy",
                    self.filtered_scene_point_cloud_in_board_coord)
            
        # compute the object center position
        object_center = np.mean(self.filtered_scene_point_cloud_in_board_coord, axis=0)
        return object_center  
    
    def _numpy_to_o3d(self, points: np.ndarray) -> o3d.geometry.PointCloud:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        return point_cloud

    def _get_point_cloud_center(sekf, point_cloud: o3d.geometry.PointCloud) -> np.ndarray:
        return np.mean(np.asarray(point_cloud.points), axis=0)

    def locate_object_position(self):
        """
        Locate the object position by matching the known the partial view point cloud of the object to 
        known fullview object model.
        """
        if self.filtered_scene_point_cloud_in_board_coord is None:
            raise ValueError("Do object partial view computation first.")

        if self.object_modeling_file_path is None or not os.path.exists(self.object_modeling_file_path):
            raise ValueError("No object modeling file is provided.")
        
        # Load point clouds 
        object_model_points = np.load(self.object_modeling_file_path)
        object_model_pcd = self._numpy_to_o3d(object_model_points)
        
        # Align and restore point clouds
        aligned_source, restored_target = align_source_to_target(object_model_pcd, 
                                                                 self._numpy_to_o3d(self.filtered_scene_point_cloud_in_board_coord))
        
        return self._get_point_cloud_center(aligned_source)
    
    def _show_pcd_in_window(self, point_cloud: o3d.geometry.PointCloud):
        # viewer = o3d.visualization.Visualizer()
        # viewer.create_window()
        # opt = viewer.get_render_option()
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(point_cloud)
        # viewer.add_geometry(pcd)
        # opt.show_coordinate_frame = True
        # opt.background_color = np.asarray([0, 0, 0])
        # viewer.run()
        # viewer.destroy_window()
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(point_cloud)
        # Visualize the reloaded point cloud with coordinate axes
        o3d.visualization.draw_geometries(
                                    [point_cloud, axes],
                                    window_name='Colored Point Cloud',
                                    width=800, height=600,
                                    left=0, top=1)
    
    def _show_points_in_window(self, point_cloud: np.ndarray):
        if len(point_cloud) == 0:
            raise ValueError("No points to visualize.")
        
        # wrong visualization with displacement coordinate axis with unknown reason
        # # viewer = o3d.visualization.Visualizer()
        # # viewer.create_window()
        # # opt = viewer.get_render_option()
        # # pcd = o3d.geometry.PointCloud()
        # # pcd.points = o3d.utility.Vector3dVector(point_cloud)
        # # viewer.add_geometry(pcd)
        # # opt.show_coordinate_frame = True
        # # opt.background_color = np.asarray([0, 0, 0])
        # # viewer.run()
        # # viewer.destroy_window()
        
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud)
        # pcd.colors = o3d.utility.Vector3dVector(point_cloud[3:6])
        # Visualize the reloaded point cloud with coordinate axes
        o3d.visualization.draw_geometries(
                                    [pcd, axes],
                                    window_name='Point Cloud',
                                    width=800, height=600,
                                    left=0, top=1)
                
    def _process_scene_pointcloud(self, x_range, y_range, z_range):
        if self.vis_scene_point_cloud_in_camera_coord:
            self._show_pcd_in_window(self.scene_point_cloud_in_camera_coord)
            
        self.scene_point_cloud_in_board_coord = \
            transform_point_cloud_from_camera_to_table(self.scene_point_cloud_in_camera_coord, 
                                                       T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                       T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        
        if self.vis_scene_point_cloud_in_board_coord:
            self._show_pcd_in_window(self.scene_point_cloud_in_board_coord)
            
        self.filtered_scene_point_cloud_in_board_coord = \
            filter_point_outside_operation_area(
                self.scene_point_cloud_in_board_coord,
                x_range=x_range,
                y_range=y_range,
                z_range=z_range)
        
        if self.vis_filtered_point_cloud_in_board_coord:
            self._show_points_in_window(self.filtered_scene_point_cloud_in_board_coord)
        
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