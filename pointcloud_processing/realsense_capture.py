import pyrealsense2 as rs
import numpy as np
import open3d as o3d

class RealSenseCapture:
    # _instance = None  # Class variable to hold the singleton instance

    # def __new__(cls, *args, **kwargs):
    #     # Check if an instance already exists
    #     if cls._instance is None:
    #         cls._instance = super(RealSenseCapture, cls).__new__(cls)
    #     return cls._instance

    def __init__(self, width=1280, height=720, depth_format=rs.format.z16, color_format=rs.format.bgr8, fps=5):
        if not hasattr(self, '_initialized'):
            self._initialized = False
            try:
                # Configure the RealSense pipeline
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.depth, width, height, depth_format, fps)
                self.config.enable_stream(rs.stream.color, width, height, color_format, fps)
                
                # Start the pipeline
                self.pipeline.start(self.config)
                self._initialized = True
            except Exception as e:
                print("Failed to initialize RealSense camera:", e)
                self._initialized = False
            # self._initialized = True  # Mark as initialized to prevent reinitialization

    # @classmethod
    # def get_instance(cls, *args, **kwargs):
    #     """Factory method to get the singleton instance."""
    #     if cls._instance is None:
    #         cls._instance = RealSenseCapture(*args, **kwargs)
    #     return cls._instance

    def reset(self):
        """Reset the pipeline if needed."""
        self.pipeline.stop()
        self.pipeline.start(self.config)

    def get_rgb_frame(self):
        """
        Retrieve a single RGB frame from the camera.

        Returns:
        - color_image: The captured RGB frame as a NumPy array.
        """
        # Wait for the next set of frames from the camera
        frames = self.pipeline.wait_for_frames()

        # Extract the color frame
        color_frame = frames.get_color_frame()

        if not color_frame:
            raise RuntimeError("Failed to retrieve RGB frame")

        # Convert the frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())

        return color_image
    
    def capture(self):
        try:
            # Capture frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                raise RuntimeError("Failed to capture frames")

            # Create point cloud object and map color to it
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Convert the point cloud to numpy arrays
            vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # Shape: (N, 3)
            tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)  # Shape: (N, 2)
            
            # Map RGB colors to the vertices
            colors = self._get_rgb_from_tex(tex, color_image)
                    
            # Create Open3D PointCloud object and assign points and colors
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(vtx)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            return pcd, color_image
            
        except Exception as e:
            raise RuntimeError(f"Could not get aligned frames: {e}")

    def _get_rgb_from_tex(self, tex_coords, color_image):
        """Helper function to map texture coordinates to RGB colors."""
        h, w, _ = color_image.shape
        # Convert normalized coordinates to pixel indices
        u = (tex_coords[:, 0] * w).astype(int)
        v = (tex_coords[:, 1] * h).astype(int)
        # Clamp the indices to ensure valid range
        u = np.clip(u, 0, w - 1)
        v = np.clip(v, 0, h - 1)
        # Extract RGB values
        rgb = color_image[v, u, :] / 255.0  # Normalize to [0, 1]
        return rgb


realsense_instance = RealSenseCapture()