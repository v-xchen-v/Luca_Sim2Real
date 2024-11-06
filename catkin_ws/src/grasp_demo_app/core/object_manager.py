import numpy as np

class ObjectManager:
    def __init__(self) -> None:
         # Object-specific configurations
        self.object_configs = {
            "names": [
                'orange_1024', 
                'coke_can_1030', 
                'realsense_box_1024',
                'cube_055_1103', 
                'bottle_coconut_1101', 
                'sunscreen_1101',
                'hammer_1102'
            ],
            "rotation_euler": [
                [-np.pi, 0, np.pi/2],    # Orange
                [-np.pi/2, np.pi/4, 0],  # Coke
                [np.pi, 0, np.pi/2],     # Realsense Box
                [-np.pi, 0, 0],          # Cube
                [0, np.pi, 0],           # Coconut Bottle
                [np.pi/2, 0, 0],         # Sunscreen
                [np.pi/2, 0, 0]          # Hammer
            ],
            "modeling_file_paths": [
                r'data/pointcloud_data/candidiate_objects/orange.npy',
                r'data/pointcloud_data/candidiate_objects/coke_can.npy',
                r'data/pointcloud_data/candidiate_objects/realsense_box.npy',
                r'data/pointcloud_data/candidiate_objects/cube_055.npy',
                r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
                r'data/pointcloud_data/candidiate_objects/sunscreen.npy',
                r'data/pointcloud_data/candidiate_objects/hammer.npy'
            ],
            "icp_rot_euler": [
                False,
                False,
                True,
                True,
                True,
                True,
                True,
            ],
            "icp_rot_euler_limits": [
                None,
                None,
                180,
                90,
                360,
                360,
                360,
            ]
        }
        
    def get_object_config(self, identifier):
        """Retrieve object configuration by name or index."""
        if isinstance(identifier, int):  # If identifier is an index
            if 0 <= identifier < len(self.object_configs["names"]):
                object_config = {
                    "name": self.object_configs["names"][identifier],
                    "rotation_euler": self.object_configs["rotation_euler"][identifier],
                    "modeling_file_path": self.object_configs["modeling_file_paths"][identifier],
                    "icp_rot_euler": self.object_configs["icp_rot_euler"][identifier],
                    "icp_rot_euler_limits": self.object_configs["icp_rot_euler_limits"][identifier]
                }
                return object_config
            else:
                raise ValueError("Index out of range for object configurations.")
        elif isinstance(identifier, str):  # If identifier is a name
            if identifier in self.object_configs["names"]:
                index = self.object_configs["names"].index(identifier)
                object_config = {
                    "name": self.object_configs["names"][index],
                    "rotation_euler": self.object_configs["rotation_euler"][index],
                    "modeling_file_path": self.object_configs["modeling_file_paths"][index],
                    "icp_rot_euler": self.object_configs["icp_rot_euler"][index],
                    "icp_rot_euler_limits": self.object_configs["icp_rot_euler_limits"][index]
                }
                return object_config
            else:
                raise ValueError("Name not found in object configurations.")
        else:
                raise TypeError("Identifier must be a string (name) or an integer (index).")