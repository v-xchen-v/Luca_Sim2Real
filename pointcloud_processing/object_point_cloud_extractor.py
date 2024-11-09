import numpy as np
from pointcloud_processing.transformations import transform_point_cloud_from_camera_to_table, invert_transform
from pointcloud_processing.table_filter import filter_point_outside_operation_area
from pointcloud_processing.pointcloud_io import load_npy_as_point_cloud
from pointcloud_processing.transformations import transform_point_cloud_from_table_to_camera
import cv2

class ObjectPointCloudExtractor:
    def __init__(self, T_calibration_board_to_camera):
        self.T_calibration_board_to_camera = T_calibration_board_to_camera
        
        # Intermediate variables
        self.object_pcd_in_board_coord = None
        self.object_pcd_in_cam_coord = None
        self.masked_pc_point_array = None
                
    def extract(self, scene_pcd, x_keep_range, y_keep_range, z_keep_range):
        scene_pcd_in_board_coord = \
            transform_point_cloud_from_camera_to_table(scene_pcd, 
                                                    T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                    T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        
        points_filtered, masked_pc_point_array = filter_point_outside_operation_area(scene_pcd_in_board_coord, 
                                                            x_range=x_keep_range, y_range=y_keep_range, z_range=z_keep_range)
        object_pcd_in_board_coord = load_npy_as_point_cloud(points_filtered)
        
        self.object_pcd_in_board_coord = object_pcd_in_board_coord
        self.masked_pc_point_array = masked_pc_point_array
        return object_pcd_in_board_coord, scene_pcd_in_board_coord, masked_pc_point_array
    
    def get_object_pcd_in_board_coord(self):
        if self.object_pcd_in_board_coord is None:
            raise ValueError("Object point cloud in board coordinate is not yet extracted.")
        
        return self.object_pcd_in_board_coord
    
    def get_object_pcd_in_cam_coord(self):
        if self.object_pcd_in_board_coord is None:
            raise ValueError("Object point cloud in board coordinate is not yet extracted.")
        
        if self.object_pcd_in_cam_coord is not None:
            return self.object_pcd_in_cam_coord
        
        object_pcd_in_cam_coord = transform_point_cloud_from_table_to_camera(self.object_pcd_in_board_coord, 
                                                   T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
                                                   T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        self.object_pcd_in_cam_coord = object_pcd_in_cam_coord
        return object_pcd_in_cam_coord
    
    def get_object_rgb_in_cam_coord(self, color_image: np.ndarray):
        if self.masked_pc_point_array is None:
            raise ValueError("Object point cloud in board coordinate is not yet extracted.")
        
        # masked_pcd_in_cam_coord = transform_point_cloud_from_table_to_camera(self.masked_pc_point_array, 
        #                                            T_depthcam_to_rgbcam=np.load(r'calibration/calibration_data/camera1/depth_to_rgb/T_depth_to_rgb.npy'),
        #                                            T_rgbcam_to_table=invert_transform(self.T_calibration_board_to_camera))
        # masked_pc_point_array_in_cam_coord = np.asarray(masked_pcd_in_cam_coord.points)
        # masked_points = masked_pc_point_array_in_cam_coord.reshape(w, h, c)
        
        # no need to transfrom to camera coord, since the point cloud ordering is captured in depth camera, naturely have info of camera whc position info
        w, h, c = color_image.shape # 720, 1280, 3
        masked_points = self.masked_pc_point_array.reshape(w, h, c)
        
        # points valued [np.nan, np.nan, np.nan] are not in the mask
        mask = ~np.isnan(masked_points).any(axis=2)
        
        # Get the min and max values of the mask
        x_min, y_min = np.min(np.where(mask), axis=1)
        x_max, y_max = np.max(np.where(mask), axis=1)

        
        # x_min, x_max, y_min, y_max = pc_mask_range
        
        # object_pcd_in_cam_coord = self.get_object_pcd_in_cam_coord()
        
        # point_cloud_array = np.asarray(object_pcd_in_cam_coord.points)
        # # Find non-zero values in each dimension
        # non_zero_points = point_cloud_array[np.all(point_cloud_array != [0, 0, 0], axis=2)]

        # # Calculate the min and max ranges in each axis for cropping
        # x_min, y_min, z_min = np.min(non_zero_points, axis=0)
        # x_max, y_max, z_max = np.max(non_zero_points, axis=0)

        # # Use this range to crop the RGB image
        # # Convert the (x, y, z) limits back to pixel coordinates
        # # u_min = int(fx * x_min / z_min + cx)
        # # u_max = int(fx * x_max / z_max + cx)
        # # v_min = int(fy * y_min / z_min + cy)
        # # v_max = int(fy * y_max / z_max + cy)
        
        # w = color_image.shape[1]
        # h = color_image.shape[0]

        # # Ensure the crop coordinates are within image bounds
        # u_min = max(0, min(x_min, w - 1))
        # u_max = max(0, min(x_max, w - 1))
        # v_min = max(0, min(y_min, h - 1))
        # v_max = max(0, min(y_max, h - 1))

        # Crop the RGB image using the calculated pixel bounds
        cropped_rgb_image = color_image[x_min:x_max, y_min:y_max]

        
        # for debugging purposes
        if True: 
            # Save or display the cropped RGB image
            cv2.imwrite("cropped_rgb_image.jpg", cropped_rgb_image)
            cv2.imshow("Cropped Image", cropped_rgb_image)
            cv2.waitKey(3000)
            cv2.destroyAllWindows()
        
        return cropped_rgb_image

    