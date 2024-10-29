import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def _show_point_cloud_window(point_cloud: o3d.geometry.PointCloud):
    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    opt = viewer.get_render_option()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud.points)
    # viewer.add_geometry(pcd)
    
    # draw the point cloud with texture color
    pcd.colors = point_cloud.colors
    viewer.add_geometry(pcd)
    opt.show_coordinate_frame = True
    opt.background_color = np.asarray([0, 0, 0])
    viewer.run()
    viewer.destroy_window()
    
# Configure the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable depth stream
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

try:
    # Wait for a coherent pair of frames: depth
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    
    if not depth_frame:
        raise RuntimeError("Could not retrieve depth frame.")

    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    print("Depth image shape:", depth_image.shape)

    # Get the intrinsic parameters of the camera
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    fx, fy = depth_intrinsics.fx, depth_intrinsics.fy
    cx, cy = depth_intrinsics.ppx, depth_intrinsics.ppy

    # Generate point cloud
    points = []
    for y in range(depth_image.shape[0]):
        for x in range(depth_image.shape[1]):
            depth = depth_image[y, x] * 0.001  # Convert to meters
            if depth > 0:
                # Back-project the pixel into 3D space
                X = (x - cx) * depth / fx
                Y = (y - cy) * depth / fy
                Z = depth
                points.append([X, Y, Z])

    # Convert to Open3D point cloud format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

finally:
    # Stop the pipeline when done
    pipeline.stop()

