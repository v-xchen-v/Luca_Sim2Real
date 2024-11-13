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
from scipy.spatial.transform import Rotation as R   
from pointcloud_processing.object_point_cloud_extractor import ObjectPointCloudExtractor
from pointcloud_processing.pointcloud_exceptions import ICPFitnessException

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
       
        # object pcd in board coord
        self.object_pcd_extractor = ObjectPointCloudExtractor(T_calibration_board_to_camera)
        
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
        self.T_calibration_board_to_camera, _ = \
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
                 T_calibration_board_to_camera: np.ndarray = None,
                 icp_fitness_threshold: float = None,
                 vis_object_icp: bool = False,
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
        self.T_calibration_board_to_camera = T_calibration_board_to_camera
        
        # intermediate data
        self.scene_point_cloud_in_board_coord = None
        self.aligned_source = None
        self.source_to_algined_rotation_matrix = None
        
        self.object_model_pcd = None
        
        # configure visualization
        self.vis_scene_point_cloud_in_camera_coord = vis_scene_point_cloud_in_cam_coord
        self.vis_scene_point_cloud_in_board_coord = vis_scene_point_cloud_in_board_coord
        self.vis_filtered_point_cloud_in_board_coord = vis_filtered_point_cloud_in_board_coord
        self.vis_filtered_point_cloud_in_cam_coord = vis_filtered_point_cloud_in_cam_coord
        self.vis_object_icp = vis_object_icp
        
        self.icp_fitness_threshold = icp_fitness_threshold
        
        
    
    
    def get_object_point_cloud(self,
                               x_range=[None, None], 
                               y_range=[None, None], 
                               z_range=[None, None]):
        self._capture_scene()
        self._load_scene()
        self._get_transform_table_to_camera(self.T_calibration_board_to_camera)
        self._process_scene_pointcloud(x_range, y_range, z_range) # [num_points, 3]
        return self.filtered_scene_point_cloud
    
    def locate_partial_view_object_position(self, 
                               x_range=[None, None], 
                               y_range=[None, None], 
                               z_range=[None, None],
                               save_filtered_point_cloud: bool = True) -> np.ndarray:
        self.get_object_point_cloud(x_range=x_range, y_range=y_range, z_range=z_range)
        
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

    def _load_object_model(self):
        # Load point clouds 
        object_model_points = np.load(self.object_modeling_file_path)
        self.object_model_pcd = self._numpy_to_o3d(object_model_points)
        
    def _icp(self):
        # Align and restore point clouds
        self.aligned_source, restored_target, self.source_to_algined_rotation_matrix,\
            fitness, rmse = align_source_to_target(self.object_model_pcd, 
                                                                 self._numpy_to_o3d(self.filtered_scene_point_cloud_in_board_coord),
                                                                 vis_aligned=self.vis_object_icp,
                                                                 switch_source_target=True)
            
        if self.icp_fitness_threshold is not None and fitness < self.icp_fitness_threshold:
            raise ICPFitnessException(f"ICP matching fitness is below threshold: {fitness}")
    
    def locate_object_position(self):
        """
        Locate the object position by matching the known the partial view point cloud of the object to 
        known fullview object model.
        """
        if self.filtered_scene_point_cloud_in_board_coord is None:
            raise ValueError("Do object partial view computation first.")

        if self.object_modeling_file_path is None or not os.path.exists(self.object_modeling_file_path):
            raise ValueError("No object modeling file is provided.")
        
        if self.object_model_pcd is None:
            self._load_object_model()
        
        if self.aligned_source is None:
            self._icp()
        
        return self._get_point_cloud_center(self.aligned_source)
    
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
        
        # destroy the window, when close
        # o3d.visualization.draw_geometries([pcd, axes])        
                
    def _process_scene_pointcloud(self, x_range, y_range, z_range):
        if self.vis_scene_point_cloud_in_camera_coord:
            self._show_pcd_in_window(self.scene_point_cloud_in_camera_coord)
            
        self.scene_point_cloud_in_board_coord = \
            transform_point_cloud_from_camera_to_table(self.scene_point_cloud_in_camera_coord, 
                                                       T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                       T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        
        if self.vis_scene_point_cloud_in_board_coord:
            self._show_pcd_in_window(self.scene_point_cloud_in_board_coord)
            
        self.filtered_scene_point_cloud_in_board_coord, _ = \
            filter_point_outside_operation_area(
                self.scene_point_cloud_in_board_coord,
                x_range=x_range,
                y_range=y_range,
                z_range=z_range)
        
        if self.vis_filtered_point_cloud_in_board_coord:
            self._show_points_in_window(self.filtered_scene_point_cloud_in_board_coord)
            
    def get_object_roi_color_image(self):
        raise NotImplementedError("This method is not implemented yet.")
        
class ObjectPoseLocator(ObjectPositionLocator):
    """
    Given a image and point cloud of scene include a table, a calibration board attach on table and a object on table,
    the ObjectPositionLocator will compute the object center position(xyz) and the rotation(xyzw) of object from 
    modeling orientation.
    """

    def __init__(self, 
                 scene_data_save_dir: str, 
                 scene_data_file_name: str, 
                 camera_intrinsics_data_dir: str,
                 R_calibration_board_to_object_placed_face_robot: np.ndarray,
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
                 T_calibration_board_to_camera: np.ndarray = None,
                 icp_rot_euler_limit:int = None,
                 icp_rot_euler_offset_after_limit = None,
                 icp_fitness_threshold: float = None,
                 vis_object_icp=False,
                 ) -> None:
        super().__init__(scene_data_save_dir,
                        scene_data_file_name,
                        camera_intrinsics_data_dir,
                        calibration_board_info,
                        object_modeling_file_path,
                        report_dir,
                        overwrite_if_exists,
                        vis_filtered_point_cloud_in_board_coord,
                        vis_scene_point_cloud_in_cam_coord,
                        vis_scene_point_cloud_in_board_coord,
                        vis_filtered_point_cloud_in_cam_coord,
                        T_calibration_board_to_camera,
                        vis_object_icp=vis_object_icp)

        """For a rotation matrix that transforms coordinates from an object’s model frame (likely
        in a virtual or CAD-based environment) to its actual orientation in the real world (specifically 
        when the object is positioned on a table, facing a robot)"""
        self.R_calibration_board_to_object_placed_face_robot = R_calibration_board_to_object_placed_face_robot
        self.icp_rot_euler_limit = icp_rot_euler_limit
        self.icp_rot_euler_offset_after_limit = icp_rot_euler_offset_after_limit
        self.icp_fitness_threshold = icp_fitness_threshold
        
        # store itermediate data
        self.R_object_placed_face_robot_to_current = None
    
    # def _process_scene_pointcloud(self):
    #     pass
    def _process_rotation_object_modeling_to_scene(self):
        """Object modeling coordinate is same as in isaac gym(not rotation at initial pose), so that
        rotation from object modeling to object in scene is same as object in sim to object in real."""
        
        """Already known the rotation from calibration board to object in real when put on table and face 
        to robot's eye.
        """
        
        """object model -> rotate to place face to robot eye -> then rotate to CURRENT real object pose in world(np.eye(4)), 
        the rotate to real object pose is what we want to know. named them A, B, C.
        W->A->B->C, then known:
        R_W_to_A = np.eye(3), assume putting object modeling in world without rotation.
        R_W_to_B = R_calibration_board_to_object_placed_face_robot, when calibration board treat as first frame (np.eye(4)).
        R_A_to_C = self.source_to_algined_rotation_matrix, do not know the right rotation matrix, how to handle?
        We want to know R_B_to_C.
        R_B_to_C = R_B_to_W * R_W_to_C = R_B_to_W * R_W_to_A * R_A_to_C 
        
        """
        # self.source_to_algined_rotation_matr = T_place_to_current?
        # R_calibration_board_to_object_placed_face_robot * source_to_algined_rotation_mat = T_board_to_current??
        # T_place_to_current = T_board_to_place placet_current
        # R_B_to_C                                   R_A_to_C  
        
        
        # R_source_to_place *place_to_currnet = R_source_to_current
        # place_to_currnet = R_source_to_place.T *  R_source_to_current
        # calibration board base coordinate how to represent modeling to placed.                               
        self.R_object_placed_face_robot_to_current = self.source_to_algined_rotation_matrix @ \
                                                     np.linalg.inv(self.R_calibration_board_to_object_placed_face_robot) # R_B_to_W
                                                     
        #                                                      # R_B_to_C                                   R_A_to_C                                 
        # self.R_object_placed_face_robot_to_current = np.linalg.inv(self.R_calibration_board_to_object_placed_face_robot) @ self.source_to_algined_rotation_matrix
        #                                               # R_B_to_W
        pass
    
    def _determine_B_axis_align_A_z(self, R_A_to_B: np.ndarray):
        # Get the third column of R, representing A's z-axis in terms of B's coordinates
        # A_z_in_B = R_A_to_B[:, 2]
        A_z_in_B = np.array([0, 0, 1])@R_A_to_B

        # Threshold for dominance (set to 0.9 for now)
        threshold = 0.9

        # Determine which axis of B aligns with A's z-axis
        if abs(A_z_in_B[0]) > threshold:
            # B\'s x-axis or negative to B\'s x-axis
            aligned_axis = 'x' if A_z_in_B[0] > 0 else '-x'
            # return 'x'
        elif abs(A_z_in_B[1]) > threshold:
            aligned_axis = 'y' if A_z_in_B[1] > 0 else '-y'
            # return 'y'
        elif abs(A_z_in_B[2]) > threshold:
            aligned_axis = 'z' if A_z_in_B[2] > 0 else '-z'
            # return 'z'
        else:
            aligned_axis = 'no single dominant axis alignment'
            return None

        print(f"A's z-axis aligns most closely with B': {aligned_axis}")
        return aligned_axis

    def _isolate_rotation_axis(self, R_A_to_B, axis='z'):
        """
        Zero out rotations around all axes except the specified one.
        Parameters:
            R (np.ndarray): Original 3x3 rotation matrix
            axis (str): Axis to keep ('x', 'y', or 'z')
        Returns:
            np.ndarray: Modified 3x3 rotation matrix with rotation isolated to the specified axis
        """
        R_new = np.zeros_like(R_A_to_B)  # Start with a zero matrix
        
        if axis == 'x' or axis == '-x':
            # Keep only the rotation around the x-axis
            R_new[0, 0] = R_A_to_B[0, 0]
            R_new[1, 1] = R_A_to_B[1, 1]
            R_new[1, 2] = R_A_to_B[1, 2]
            R_new[2, 1] = R_A_to_B[2, 1]
            R_new[2, 2] = R_A_to_B[2, 2]
            
        elif axis == 'y' or axis == '-y':
            # Keep only the rotation around the y-axis
            R_new[0, 0] = R_A_to_B[0, 0]
            R_new[0, 2] = R_A_to_B[0, 2]
            R_new[1, 1] = R_A_to_B[1, 1]
            R_new[2, 0] = R_A_to_B[2, 0]
            R_new[2, 2] = R_A_to_B[2, 2]
            
        elif axis == 'z' or axis == '-z':
            # Keep only the rotation around the z-axis
            R_new[0, 0] = R_A_to_B[0, 0]
            R_new[0, 1] = R_A_to_B[0, 1]
            R_new[1, 0] = R_A_to_B[1, 0]
            R_new[1, 1] = R_A_to_B[1, 1]
            R_new[2, 2] = R_A_to_B[2, 2]
            
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'")
            
        return R_new

    def _limit_rotation_to_axis(self, rotation_matrix, axis, limit=90, offset_after_limit=0):
        """Assume the object is rotation invariant around the z-axis every 90 degrees."""
        # should be 'xyz', rot z->rot y->rot x, to avoid guess of x, y, since we known
        # it have only rot on z-axis
        zyx = R.from_matrix(rotation_matrix).as_euler('ZYX', degrees=True)
        def _limit_rotation_angle(angle, limit, offset_after_limit):
            # if angle is positive, +360
            if angle < 0:
                angle += 360
            
            if limit == 180:
                # -90, +90
                # Assuming `angle` is in degrees
                angle = angle % 180  # First, bring the angle within the range [0, 180)

                # Map to the range [-90, 90]
                if angle > 90:
                    angle -= 180
            if limit == 90:
                angle = angle % 90
                
                if angle > 45:
                    angle -= 90
            else:
                # if angle is greater than limit, %limit
                if angle > limit:
                    angle %= limit
                    
                angle += offset_after_limit
            
            
                
            return angle
        
        limit_dimension = {'x': 2, 'y': 1, 'z': 0}
        # xyz[0] = 0
        # xyz[1] = 0
        # the other two axis should be 0
        # for other_axis in np.delete(np.array([0, 1, 2]), [limit_dimension[axis]]):
        #     xyz[other_axis] = 0
        zyx[limit_dimension[axis]] = _limit_rotation_angle(zyx[limit_dimension[axis]], limit,
                                                           offset_after_limit=offset_after_limit)
        
        # hope it rot to the -z of calibration board, so that easy to operate by right arm
        # if axis in ['-x', '-y', '-z']:
        #     xyz[limit_dimension[axis]] = -xyz[limit_dimension[axis]]
        #     pass 
        #     # axis = axis[1:]
        # else:
        # if limit < 360:
        #     angle = zyx[limit_dimension[axis]]
        #     angle %= limit
        #     angle += offset_after_limit
            # angle -= limit
            # zyx[limit_dimension[axis]] = angle
            
        
        print(f"Continue rotation to {zyx[limit_dimension[axis]]} degrees around the board's {axis}-axis, \n\
              or {-zyx[limit_dimension[axis]]} degrees around the board's -{axis} axis, \n\
              after place object face to robot, then got current pose")
        
        return R.from_euler('ZYX', zyx, degrees=True).as_matrix()
        
    def _locate_object_orientation(self):
        self._process_rotation_object_modeling_to_scene()
        
        # if we only care the rotation across world(table, calibration board)'s z axis from place to current
        # z_in_world = np.array([0, 0, 1])
        R_world_to_placed_object = self.R_calibration_board_to_object_placed_face_robot
        # z_placed_object = R_world_to_placed_object @ z_in_world
        
        aligned_axis = self._determine_B_axis_align_A_z(R_world_to_placed_object)
        
        if aligned_axis is None:
            raise ValueError("No single dominant axis alignment. Please check the R_object_placed_face_robot_to_calibration_board.")
        
        # if aligned_axis in ['-x', '-y', '-z']:
        #     aligned_axis = '-z'
        # elif aligned_axis in ['x', 'y', 'z']:
        #     aligned_axis = 'z'
        # aligned_axis = 'z'
        
        
        R_worldz_object_placed_face_robot_to_current = self._isolate_rotation_axis(
            self.R_object_placed_face_robot_to_current,
            axis='z')
        
        print("icp_rot_euler_limit: ", self.icp_rot_euler_limit)
        R_limited_worldz_object_placed_face_robot_to_current = self._limit_rotation_to_axis(
            R_worldz_object_placed_face_robot_to_current,
            axis='z', 
            limit=self.icp_rot_euler_limit,
            offset_after_limit=self.icp_rot_euler_offset_after_limit)
        z_degree = R.from_matrix(R_limited_worldz_object_placed_face_robot_to_current).as_euler('xyz', degrees=True)[2]
        print(f'z_degree: {z_degree}')
        
        dimension = {'x': 0, 'y': 1, 'z': 2,
                     '-x': 0, '-y': 1, '-z': 2}
        if aligned_axis in ['-x', '-y', '-z']:
            euler_degrees = [0, 0, 0]
            euler_degrees[dimension[aligned_axis]] = -z_degree
            aligned_axis = aligned_axis[1:]
            # euler_degrees[dimension[aligned_axis]] = z_degree # not sure???
        else:
            euler_degrees = [0, 0, 0]
            euler_degrees[dimension[aligned_axis]] = z_degree
        R_placed_to_current = R.from_euler('XYZ', euler_degrees, degrees=True).as_matrix()
        print(f'continue rotate across object\'s {aligned_axis} axis {euler_degrees[dimension[aligned_axis]]} degrees to current pose')
        R3=R_world_to_placed_object @ R_placed_to_current
        # R.from_euler('XYZ', [])
        
        # place_xyz = R.from_matrix(self.R_calibration_board_to_object_placed_face_robot).as_euler('xyz', degrees=True)
        
        
        # R_board_to_object_current = R_limited_worldz_object_placed_face_robot_to_current @ R_world_to_placed_object
        # rotx
        # R_A_to_B = 1,0,0; 0,0,1, 0,-1,0
        # R_limited_worldz_object_placed_face_robot_to_current2 = R_world_to_placed_object @ R_limited_worldz_object_placed_face_robot_to_current @ np.linalg.inv(R_world_to_placed_object)
        # R_board_to_object_current = R_limited_worldz_object_placed_face_robot_to_current @ R_world_to_placed_object # ???, not sure here
        # R2 = R_limited_worldz_object_placed_face_robot_to_current2 @ R_world_to_placed_object
        # R3 = self.calibration@(np.linalg.inv(self.R_calibration_board_to_object_placed_face_robot) @ R_limited_worldz_object_placed_face_robot_to_current)
        # R3 = self.R_calibration_board_to_object_placed_face_robot @ (R_limited_worldz_object_placed_face_robot_to_current @ np.linalg.inv(self.R_calibration_board_to_object_placed_face_robot) )
        return R3
    
    def locate_object_pose(self,  
                            x_range=[None, None], 
                            y_range=[None, None], 
                            z_range=[None, None],
                            save_filtered_point_cloud: bool = True):
        # locate object position first
        self.locate_partial_view_object_position(
            x_range=x_range, y_range=y_range, z_range=z_range,
            save_filtered_point_cloud=save_filtered_point_cloud)
        translation = self.locate_object_position()
        
        if self.source_to_algined_rotation_matrix is None:
            raise ValueError("No rotation matrix is computed.")

        orientation = self._locate_object_orientation()
        
        # T_board_to_object_current
        pose = np.eye(4)
        pose[:3, :3] = orientation
        pose[:3, 3] = translation
        return pose
        
        