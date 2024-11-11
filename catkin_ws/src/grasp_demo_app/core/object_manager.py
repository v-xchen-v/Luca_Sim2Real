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
                "icp_rot_euler_offset_after_limit": 0,
                "first_n_steps": 70,
                "grasp_traj_hz": 4,
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy",
                "hand_offset_at_n_step": None, # start af offset at step n, when object is upper than table
                "hand_offset": 0, # degree
                "pregrasp_tscale": 1.5,                
                "hand_preoffset_for_all_steps": 0,
            },
            {
                "name": "cube_055_1103",
                "rotation_euler": [-np.pi, 0, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/cube_055.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 90,
                "icp_rot_euler_offset_after_limit": -90,
                "first_n_steps": 70,
                "grasp_traj_hz": 6, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy",
                "hand_offset_at_n_step": 50, # start af offset at step n, when object is upper than table
                "hand_offset": 6, # degree
                "pregrasp_tscale": 1.3,
                "hand_preoffset_for_all_steps": 0,
            },
            {
                "name": "realsense_box_1024",
                "rotation_euler": [np.pi, 0, 0], #np.pi/2],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/realsense_box.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 180,
                "icp_rot_euler_offset_after_limit": 0, # must be limit*n, n could be 0, 1, 2
                "first_n_steps": 70,
                "grasp_traj_hz": 6, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-0.npy",
                "hand_offset_at_n_step": 40, # start af offset at step n, when object is upper than table
                "hand_offset": 3, # degree 
                "pregrasp_tscale": 1.2,
                "hand_preoffset_for_all_steps": 0,
            },
            # {
            #     "name": "bottle_coconut_1101",
            #     "rotation_euler": [0, np.pi, -np.pi/2],
            #     "modeling_file_path": r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
            #     "icp_rot_euler": True,
            #     "icp_rot_euler_limit": 180, # 360
            #     "icp_rot_euler_offset_after_limit": 0, # 180
            #     "first_n_steps": 70,
            #     "grasp_traj_hz": 3, 
            #     "hand_lift_offset": 0,    
            #     "sim_traj_file_name": "step-4400.npy",
            #     "hand_offset_at_n_step": 45, # start af offset at step n, when object is upper than table
            #     "hand_offset": 6, # degree 
            #     "pregrasp_tscale": 1.2,
            # },
            {
                "name": "tape_measure_1105",
                "rotation_euler": [np.pi, 0, np.pi/2],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/tape_measure.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 180,
                "icp_rot_euler_offset_after_limit": 0,
                "first_n_steps": 80,
                "grasp_traj_hz": 4, 
                "hand_lift_offset": 0,  
                "sim_traj_file_name": "step-0.npy",
                "hand_offset_at_n_step": 40, # start af offset at step n, when object is upper than table
                "hand_offset": 8, # degree 
                "pregrasp_tscale": 1.2,                
                "hand_preoffset_for_all_steps": 0,
            },
            {
                "name": "bottle_coconut_1105",
                "rotation_euler": [-np.pi/2, np.pi/2, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/bottle_coconut.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 90, # 360
                "icp_rot_euler_offset_after_limit": 0, # 180
                "first_n_steps": 120,
                "grasp_traj_hz": 6, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-800.npy",
                "hand_offset_at_n_step": 70, # start af offset at step n, when object is upper than table
                "hand_offset": 25, # degree 
                "pregrasp_tscale": 1.2,
                "hand_preoffset_for_all_steps": -10, #degree
            },
            {
                "name": "coke_can_1104",
                "rotation_euler": [-np.pi/2, -np.pi/2-np.pi/2+np.pi/3+0/180*np.pi, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/coke_can_new.npy',
                "icp_rot_euler": False,
                "icp_rot_euler_limit": None,
                "icp_rot_euler_offset_after_limit": 0,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,    
                "sim_traj_file_name": "step-4000.npy",
                "hand_offset_at_n_step": 150, # start af offset at step n, when object is upper than table
                "hand_offset": 30, # degree 
                "pregrasp_tscale": 1.2,               
                "hand_preoffset_for_all_steps": 0,
            },
            {
                "name": "duck_1104",
                "rotation_euler": [-np.pi/2, np.pi/2, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/duck.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "icp_rot_euler_offset_after_limit": 0,
                "first_n_steps": 200,
                "grasp_traj_hz": 2, 
                "hand_lift_offset": 0,  
                "sim_traj_file_name": "step-0.npy",
                "pregrasp_tscale": 1.2,                
                "hand_preoffset_for_all_steps": 0,
            },
            # {
            #     "name": "hammer_1102",
            #     "rotation_euler": [np.pi/2, 0, 0],
            #     "modeling_file_path": r'data/pointcloud_data/candidiate_objects/hammer.npy',
            #     "icp_rot_euler": True,
            #     "icp_rot_euler_limit": 360,
            #     "icp_rot_euler_offset_after_limit": 0,
            #     "first_n_steps": 200,
            #     "grasp_traj_hz": 2, 
            #     "hand_lift_offset": 0,  
            #     "pregrasp_tscale": 1.2,                
            #     "hand_preoffset_for_all_steps": 0,
            # },
            # {
            #     "name": "coke_can_1030",
            #     "rotation_euler": [-np.pi/2, np.pi/4, 0],
            #     "modeling_file_path": r'data/pointcloud_data/candidiate_objects/coke_can.npy',
            #     "icp_rot_euler": False,
            #     "icp_rot_euler_limit": None,
            #     "icp_rot_euler_offset_after_limit": 0,
            #     "first_n_steps": 200,
            #     "grasp_traj_hz": 2, 
            #     "hand_lift_offset": 0,    
            #     "sim_traj_file_name": "step-0.npy",
            #     "pregrasp_tscale": 1.2,
            # },
            {
                "name": "sunscreen_1101",
                "rotation_euler": [np.pi/2, 0, 0],
                "modeling_file_path": r'data/pointcloud_data/candidiate_objects/sunscreen.npy',
                "icp_rot_euler": True,
                "icp_rot_euler_limit": 360,
                "icp_rot_euler_offset_after_limit": 0,
                "first_n_steps": 200,
                "grasp_traj_hz": 2,
                "hand_lift_offset": 0,     
                "sim_traj_file_name": "step-0.npy",
                "pregrasp_tscale": 1.2,                
                "hand_preoffset_for_all_steps": 0,
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