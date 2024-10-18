import numpy as np
import json
from pytransform3d.transformations import plot_transform
import matplotlib.pyplot as plt

# Dummy logic of trajectory adaptor
class TrajectoryAdaptor:
    def __init__(self, calibration_file):
        """
        Initialize TrajectoryAdaptor with pre-computed calibration data.
        """
        self.calibration_data = self.load_calibration_data(calibration_file)
        self.camera_intrinsic = self.calibration_data.get("camera_intrinsic")
        self.T_camera_to_robot = np.array(self.calibration_data.get("T_camera_to_robot"))
        self.T_table_to_camera = None  # To be computed in dynamic calibration

    def load_calibration_data(self, calibration_file):
        """
        Load pre-computed calibration data from a JSON file.
        """
        with open(calibration_file, 'r') as f:
            return json.load(f)

    def compute_relative_transformation(self, T_source, T_target):
        """
        Compute the relative transformation between two frames.
        T_source: 4x4 matrix representing the source frame.
        T_target: 4x4 matrix representing the target frame.
        """
        T_source_inv = np.linalg.inv(T_source)
        return T_source_inv @ T_target

    def capture_frame_for_calibration(self, T_table_to_camera):
        """
        Capture a frame to compute the table to camera transformation.
        """
        # Example: Assume T_table_to_camera is computed from captured data
        self.T_table_to_camera = np.array(T_table_to_camera)
        print("Captured and set table-to-camera transformation.")

    def map_sim_to_real(self, T_sim_world_to_table):
        """
        Map a transformation from the simulator to the real world.
        T_sim_world_to_table: 4x4 matrix of simulator’s table relative to simulator world.
        """
        if self.T_table_to_camera is None:
            raise ValueError("Table-to-Camera transformation not set. Perform calibration first.")

        # Compute real world mapping: camera-to-robot and table-to-camera are known
        T_table_to_robot = self.compute_relative_transformation(self.T_table_to_camera, self.T_camera_to_robot)
        print("Computed Table-to-Robot transformation:\n", T_table_to_robot)

        # Now compute the mapping from simulator to real world
        T_sim_to_real = T_table_to_robot @ T_sim_world_to_table
        return T_sim_to_real

    def visualize_frames(self, frames):
        """
        Visualize multiple frames in a 3D plot.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for name, T in frames.items():
            plot_transform(ax=ax, A2B=T, s=0.2, name=name)

        # Set plot limits and labels
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([0, 2])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        plt.title("Coordinate Frames Visualization")
        plt.show()

# Example Usage
if __name__ == "__main__":
    # Initialize the trajectory adaptor with pre-computed calibration data
    adaptor = TrajectoryAdaptor("calibration_data.json")

    # Capture frame and compute table-to-camera transformation (mock example)
    T_table_to_camera = [
        [1, 0, 0, 0.5],
        [0, 1, 0, 0.3],
        [0, 0, 1, 0.2],
        [0, 0, 0, 1]
    ]
    adaptor.capture_frame_for_calibration(T_table_to_camera)

    # Define the simulator world to table transformation (mock example)
    T_sim_world_to_table = np.array([
        [0, -1, 0, 1],
        [1, 0, 0, 2],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Map simulator transformations to real-world coordinates
    T_sim_to_real = adaptor.map_sim_to_real(T_sim_world_to_table)
    print("Mapped Simulator to Real-World Transformation:\n", T_sim_to_real)

    # Visualize the transformations
    frames = {
        "Camera to Robot": adaptor.T_camera_to_robot,
        "Table to Camera": adaptor.T_table_to_camera,
        "Sim World to Table": T_sim_world_to_table,
        "Sim to Real": T_sim_to_real
    }
    adaptor.visualize_frames(frames)
