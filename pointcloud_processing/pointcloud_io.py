""" Loads the point cloud from sensors or files."""
import numpy as np
import open3d as o3d
import os
import pyrealsense2 as rs
import cv2
from .realsense_capture import realsense_instance
import time

def load_npy_file_as_point_cloud(path: str) -> o3d.geometry.PointCloud:
    """
    Load the point cloud from the given path.

    Args:
    - path: The path to the point cloud file. The file must be in NumPy format.
    """
    point_cloud = np.load(path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    
    return pcd

def load_npy_as_point_cloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Load the point cloud from the given path.

    Args:
    - points: The point cloud as a NumPy array.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    return pcd

def load_point_cloud(path: str) -> o3d.geometry.PointCloud:
    """
    Load the point cloud from the given path.

    Args:
    - path: The path to the point cloud file. The file must be in PCD format or PLY format.
    
    Returns:
    - points: The loaded point cloud as a NumPy array.
    """
    # if file not exists, throw error
    if not os.path.exists(path):
        raise ValueError("point cloud file not found")
    
    point_cloud = o3d.io.read_point_cloud(path)
    
    return point_cloud
        
def load_point_cloud_as_numpy(path: str) -> np.ndarray:
    """
    Load the point cloud from the given path.

    Args:
    - path: The path to the point cloud file. The file must be in PCD format or PLY format.
    
    Returns:
    - points: The loaded point cloud as a NumPy array.
    """
    point_cloud = o3d.io.read_point_cloud(path)
    points = np.asarray(point_cloud.points)
    
    return points
        
def _save_realsense_point_cloud(pcd, color_image, save_dir: str, file_name: str, overwrite_if_exists=False) -> None:
    """
    Save the point cloud captured from the Intel RealSense D455 camera. The point cloud is saved as a PCD file, and a PLY file.
    And the color image is saved as a PNG file for calibration task.
    """
    exts = ['.pcd']
    
    # create the directory if it doesn't exist
    os.makedirs(save_dir, exist_ok=True)
    
    # check if at least one file with the ext already exists
    for ext in exts:
        path = os.path.join(save_dir, file_name + ext)
        if os.path.exists(path) and not overwrite_if_exists:
            raise FileExistsError(f"File already exists at {path}")
        
    # # save the point cloud as a NumPy array
    # np.save(os.path.join(save_dir, file_name + '.npy'), point_cloud)
        
    # # save the point cloud as a PLY file
    # ## Create save_to_ply object
    # ply = rs.save_to_ply(f"{save_dir}/{file_name}.ply")

    # ## Set options to the desired values
    # ## In this option settings, we'll generate a textual PLY with normals (mesh is already created by default)
    # ply.set_option(rs.save_to_ply.option_ply_binary, False)
    # ply.set_option(rs.save_to_ply.option_ply_normals, True)

    # print(f"Saving to {file_name}.ply...")
    # ## Apply the processing block to the frameset which contains the depth frame and the texture
    # ply.process(rs_frames)
    # print("Done")
    
    # save the point cloud as a PCD file
    ## read ply file and write to pcd file
    print(f"Saving to {file_name}.pcd...")
    o3d.io.write_point_cloud(f"{save_dir}/{file_name}.pcd", pcd)
    print("Done")
    
    # save the color image
    print(f"Saving color image to {file_name}.png...")
    cv2.imwrite(f"{save_dir}/{file_name}.png", color_image)
    
    # Add a short delay to ensure file system writes complete
    time.sleep(0.3)
    print("Done")
        
# def save_point_cloud_from_sensors(self) -> np.ndarray:
#     """
#     Load the point cloud from the connected sensors.

#     Returns:
#     - point_cloud: The captured point cloud as a NumPy array.
#     """
#     save_point_cloud_from_realsense()

def _image_and_point_cloud_exists(save_dir: str, file_name: str) -> bool:
    exts = ['.pcd', '.ply', '.png']
    
    # check if at least one file with the ext already exists
    for ext in exts:
        path = os.path.join(save_dir, file_name + ext)
        if not os.path.exists(path):
            return False
    
    return True

def _show_point_cloud_window(point_cloud: o3d.geometry.PointCloud):
    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    opt = viewer.get_render_option()
    pcd = o3d.geometry.PointCloud()
    vtx = np.asanyarray(point_cloud)  # XYZ
    flipy_points = vtx
    # # flipy_points[:, 1] *= -1 
    # # flipy_points[:, 2] *= -1 
    # # flipy_points[:, 0] *= 0.001   
    # R_x_90 = np.array([[1, 0, 0],
    #                [0, 0, -1],
    #                [0, 1, 0]])
    # R_y_90 = np.array([[0, 0, 1],
    #                [0, 1, 0],
    #                [-1, 0, 0]])
    # R_z_90 = np.array([[0, -1, 0],
    #                [1, 0, 0],
    #                [0, 0, 1]])
    # flipy_points = flipy_points @ R_z_90.T
    pcd.points = o3d.utility.Vector3dVector(flipy_points)
    # viewer.add_geometry(pcd)
    
    # draw the point cloud with texture color
    pcd.colors = point_cloud.colors
    viewer.add_geometry(pcd)
    opt.show_coordinate_frame = True
    opt.background_color = np.asarray([0, 0, 0])
    viewer.run()
    viewer.destroy_window()
      
# def get_image_and_point_cloud_from_realseanse():
#     # Configure the RealSense pipeline
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#     pipeline.start(config)

#     try:
#         # Capture frames: depth and color
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()

        
#         if not depth_frame or not color_frame:
#             raise RuntimeError("Failed to capture frames")

#         # Create point cloud object and map color to it
#         pc = rs.pointcloud()
#         pc.map_to(color_frame)
#         points = pc.calculate(depth_frame)
        
#         # Convert images to numpy arrays
#         color_image = np.asanyarray(color_frame.get_data())
#         # cv2.imwrite(f"output/{args.filename}.png", color_image)
        
#         # Convert the point cloud to numpy arrays
#         vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # Shape: (N, 3)
#         tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)  # Shape: (N, 2)
        
#         # Helper function to map texture coordinates to RGB colors
#         def get_rgb_from_tex(tex_coords, color_image):
#             """Convert normalized texture coordinates to RGB values."""
#             h, w, _ = color_image.shape
#             # Convert normalized coordinates to pixel indices
#             u = (tex_coords[:, 0] * w).astype(int)
#             v = (tex_coords[:, 1] * h).astype(int)
#             # Clamp the indices to ensure valid range
#             u = np.clip(u, 0, w - 1)
#             v = np.clip(v, 0, h - 1)
#             # Extract RGB values
#             rgb = color_image[v, u, :] / 255.0  # Normalize to [0, 1]
#             return rgb

#         # Map RGB colors to the vertices
#         colors = get_rgb_from_tex(tex, color_image)
                

                    
#         # Create Open3D PointCloud object and assign points and colors
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(vtx)
#         pcd.colors = o3d.utility.Vector3dVector(colors)
#         return pcd, color_image
        
#     except Exception as e:
#         raise RecursionError(f"Could not get aligned frames: {e}")
#     finally:
#         pipeline.stop()
#         # raise RuntimeError("Could not get aligned frames")
#         # return None, None

def get_image_and_point_cloud_from_realseanse():
    pcd, color_image = realsense_instance.capture()
    return pcd, color_image

def save_image_and_point_cloud_from_realsense(save_dir: str, file_name: str, overwrite_if_exists):
    """
    Capture a point cloud from the Intel RealSense D455 camera.

    Returns:
    - points: The captured point cloud.
    - color_image: The corresponding color image.
    """
    if overwrite_if_exists==False and _image_and_point_cloud_exists(save_dir, file_name):
        return None, None
    
    pcd, color_image = get_image_and_point_cloud_from_realseanse()
    

    # debugging purpose of visualizing the point cloud and the coordinate axes
    if False:
        ##Create a coordinate frame (axes) centered at the origin
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([pcd, axes])

    # o3d.io.write_point_cloud(f"/home/yichao/Documents/repos/Luca_Transformation/data/scene_data/test_scene_data/test_scene.pcd", pcd)
    # points.export_to_ply('./point_cloud.ply', color_frame)
    
    _save_realsense_point_cloud(pcd, color_image, save_dir, file_name, overwrite_if_exists)
    
    return pcd, color_image

    
