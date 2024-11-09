import numpy as np

class ObjectManager:
    def __init__(self) -> None:
        # Object-specific configurations as a list of dictionaries
        self.object_configs = [
            {
                "name": "orange_1024",
                "rotation_euler": [-np.pi, 0, np.pi/2],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/orange.npy',
                "icp_rot_euler": False,
                "icp_rot_euler_limit": None,
                "first_n_steps": 100,
                "grasp_traj_hz": 4,
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "realsense_box_1024",
                "rotation_euler": [np.pi, 0, np.pi/2],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/realsense_box.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 180,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "cube_055_1103",
                "rotation_euler": [-np.pi, 0, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/cube_055.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 90,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "duck_1104",
                "rotation_euler": [-np.pi/2, np.pi/2, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/duck.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,  
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "tape_measure_1105",
                "rotation_euler": [np.pi, np.pi/2, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/tape_measure.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 180,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,  
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "hammer_1102",
                "rotation_euler": [np.pi/2, 0, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/hammer.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,  
            },
            {
                "name": "coke_can_1030",
                "rotation_euler": [-np.pi/2, np.pi/4, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/coke_can.npy',
                "icp_rot_euler": False,
                "icp_rot_euler_limit": None,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "bottle_coconut_1101",
                "rotation_euler": [0, np.pi, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy" 
            },
            {
                "name": "sunscreen_1101",
                "rotation_euler": [np.pi/2, 0, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/sunscreen.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "first_n_steps": 200,
                "grasp_traj_hz": 2,
                "hand_lift_offset": 0,     
                "sim_traj_file_name": "step-0.npy" 
            },
        ]
        
    def get_object_config(self, identifier):
        """
        Retrieve object configuration by name or index.
        
        Parameters:
            identifier (str or int): The name (str) or index (int) of the object.

        Returns:
            dict: Configuration of the specified object.
        
        Raises:
            ValueError: If the identifier is not found.
            TypeError: If the identifier type is invalid.
        """
        if isinstance(identifier, int):  # If identifier is an index
            if 0 <= identifier < len(self.object_configs):
                return self.object_configs[identifier]
            else:
                raise ValueError("Index out of range for object configurations.")
        elif isinstance(identifier, str):  # If identifier is a name
            for config in self.object_configs:
                if config["name"] == identifier:
                    return config
            raise ValueError("Name not found in object configurations.")
        else:
            raise TypeError("Identifier must be a string (name) or an integer (index).")

if __name__ == "__main__":
    manager = ObjectManager()
    config_by_name = manager.get_object_config("coke_can_1030")
    config_by_index = manager.get_object_config(1)

    print("Config by name:", config_by_name)
    print("Config by index:", config_by_index)