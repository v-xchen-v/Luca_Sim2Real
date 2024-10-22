""" Loads the point cloud from sensors or files."""
import numpy as np
import open3d as o3d
import os
import pyrealsense2 as rs

# def load_point_cloud(self, path: str) -> np.ndarray:
#     """
#     Load the point cloud from the given path.

#     Args:
#     """
#     return np.load(path)
        
        
def _save_realsense_point_cloud(rs_points, rs_frames, save_dir: str, file_name: str, overwrite_if_exists=False) -> None:
    """
    Save the point cloud captured from the Intel RealSense D455 camera. The point cloud is saved as a PCD file, and a PLY file.
    """
    exts = ['.pcd', '.ply']
    
    # create the directory if it doesn't exist
    os.makedirs(save_dir, exist_ok=True)
    
    # check if at least one file with the ext already exists
    for ext in exts:
        path = os.path.join(save_dir, file_name + ext)
        if os.path.exists(path) and not overwrite_if_exists:
            raise FileExistsError(f"File already exists at {path}")
        
    # # save the point cloud as a NumPy array
    # np.save(os.path.join(save_dir, file_name + '.npy'), point_cloud)
        
    # save the point cloud as a PLY file
    ## Create save_to_ply object
    ply = rs.save_to_ply(f"{save_dir}/{file_name}.ply")

    ## Set options to the desired values
    ## In this option settings, we'll generate a textual PLY with normals (mesh is already created by default)
    ply.set_option(rs.save_to_ply.option_ply_binary, False)
    ply.set_option(rs.save_to_ply.option_ply_normals, True)

    print(f"Saving to {file_name}.ply...")
    ## Apply the processing block to the frameset which contains the depth frame and the texture
    ply.process(rs_frames)
    print("Done")
    
    # save the point cloud as a PCD file
    ## read ply file and write to pcd file
    o3d.io.write_point_cloud(f"{save_dir}/{file_name}.pcd", 
                             o3d.io.read_point_cloud(f"{save_dir}/{file_name}.ply"))
        
# def save_point_cloud_from_sensors(self) -> np.ndarray:
#     """
#     Load the point cloud from the connected sensors.

#     Returns:
#     - point_cloud: The captured point cloud as a NumPy array.
#     """
#     save_point_cloud_from_realsense()

    
def save_point_cloud_from_realsense(save_dir: str, file_name: str, overwrite_if_exists):
    """
    Capture a point cloud from the Intel RealSense D455 camera.

    Returns:
    - points: The captured point cloud.
    - color_image: The corresponding color image.
    """
    import pyrealsense2 as rs
    
    # Declare pointcloud object, for calculating pointclouds and texture mappings
    pc = rs.pointcloud()
    # We want the points object to be persistent so we can display the last cloud when a frame drops
    points = rs.points()

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()
    config = rs.config()
    # Enable depth stream
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    # Enable color stream
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming with chosen configuration
    pipe.start(config)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others (here to color frame)
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    try:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        
        # Align the depth frame to the color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            raise RuntimeError("Could not get color frame")

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        # cv2.imwrite(f"output/{args.filename}.png", color_image)
        
        # Check if both frames are valid
        if not aligned_depth_frame or not color_frame:
            raise RuntimeError("Could not get aligned frames")
        
        # Map the color frame to the point cloud
        pc.map_to(color_frame)
        
        # Calculate the point cloud with the aligned depth frame
        points = pc.calculate(aligned_depth_frame)
        
        _save_realsense_point_cloud(points, frames, save_dir, file_name, overwrite_if_exists)
        
        return points, color_image
    finally:
        pipe.stop()
        return None, None