"""The GraspAndPlaceExecutor class is responsible for executing the grasp and place actions."""


import numpy as np
from .robot_command_manager import RobotCommandManager
import rospy 
from scipy.spatial.transform import Rotation as R
from robot_arm_py.arm_ik import Arm_IK
import os

class GraspAndPlaceExecutor:
    def __init__(self, arm_right_urdf_path) -> None:
        # init a node here
        rospy.init_node("robot_command_manager", anonymous=True)
        
        # joint angles of the home position, order in [joint1, ..., joint7]
        ## Represent the home position of the robot
        self.home_position_angles = np.array([-41.3, -8.7, -27.2, 74.0, 71.8, -43.3, 20.0]) /180 * np.pi
        # TODO: give a dummy place at first, replace to real position later
        self.preplace_position = np.array([-41.3, -24.9, -44.6, 77.3, 68.8, -12.7, 6.2]) /180*np.pi
        
        # command manager
        self.robot_comand_manager = RobotCommandManager()
        
        # arm_right_urdf_path=f"/home/xichen/Documents/repos/Luca_Sim2Real/catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R/Realman_Inspire_R.urdf"
        # arm_right_urdf_package_dirs=[
        # f"/home/xichen/Documents/repos/Luca_Sim2Real/catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R"]
        arm_right_urdf_package_dirs = [os.path.dirname(arm_right_urdf_path)]
        self.arm_ik = Arm_IK(arm_right_urdf_path, arm_right_urdf_package_dirs, 
                             target_frame_name="R_hand_base_link_in_sim")

    def goto_home(self, table_obstacle):
        """Return to the home position"""            
        rospy.sleep(2)
        home_hand_joint_angles = [0, 0, 0, 0, 0, 0]
        home_eef_pose = self.arm_ik.fk(self.home_position_angles)
        try:
            self.robot_comand_manager.moveto_pose_with_moveit_plan(
                home_eef_pose, 
                home_hand_joint_angles, 
                table_obstacle=table_obstacle)
    
        except Exception as ServiceException:
            print("Failed to reach the pregrasp position by moveit.")
            raise ServiceException     
        
        print('Robot returned to home position.')
    
    def goto_preplace(self, table_obstacle):
        """Approach the target position for placing"""
        rospy.sleep(2)
        # TODO: move arm but in progress, do not move the hand, -1 for not moving hand?
        preplace_eef_pose = self.arm_ik.fk(self.preplace_position)
        try:
            self.robot_comand_manager.moveto_pose_with_moveit_plan(
                preplace_eef_pose, 
                None, 
                table_obstacle=table_obstacle)
    
        except Exception as ServiceException:
            print("Failed to reach the preplace position by moveit.")
            raise ServiceException     

    def goto_pregrasp(self, pregrasp_eef_pose_matrix, pregrasp_hand_angles, table_obstacle):
        """Control the speed and position of pregrasp position, it matters for the grasp success"""
        """Optional"""
        """Speed control, softly reach the rl traj start point."""
        """Approach the target position"""
        matrix_to_xyzq = lambda matrix: np.concatenate([matrix[:3, 3], R.from_matrix(matrix[:3, :3]).as_quat()])
        pregrasp_eef_pose = np.array(matrix_to_xyzq(pregrasp_eef_pose_matrix))
        
        # if type != 'moveit':
        #     tscaled_first_traj_point = np.concatenate((pregrasp_eef_pose, pregrasp_hand_angles))
        #     print("goto_pregrasp by spliting joint angles")
        #     self.robot_comand_manager.execute_trajectory([tscaled_first_traj_point], hz=hz)
            
        print("goto_pregrasp by moveit")
        try:
            moveit_response = self.robot_comand_manager.moveto_pose_with_moveit_plan(
                pregrasp_eef_pose, 
                pregrasp_hand_angles, 
                table_obstacle=table_obstacle)
            print(f'moveit_response: {moveit_response}')
    
        except Exception as ServiceException:
            print("Failed to reach the pregrasp position by moveit.")
            raise ServiceException         
                   
        print('Robot is reaching the pregrasp position.')
        # import time
        # time.sleep(5)

    def _execute_rl_trajectory(self, real_traj_path, first_n_steps, hz, 
                               hand_offset_at_n_step, hand_offset, hand_preoffset_for_all_steps):
        """Execute the RL trajectory"""
        
        # TODO: compute first n steps by  flag
        if first_n_steps is None:
            first_n_steps = len(real_traj)
            
        real_traj = np.load(real_traj_path)
        self.robot_comand_manager.execute_trajectory(real_traj[:first_n_steps], hz=hz, 
                                                     hand_offset_at_n_step=hand_offset_at_n_step, 
                                                     hand_offset=hand_offset,
                                                     hand_preoffset_for_all_steps=hand_preoffset_for_all_steps)
        print('Grasp trajectory executed.')

    def grasp(self, real_traj_path, first_n_steps, hz, hand_offset_at_n_step=None, hand_offset=0, hand_preoffset_for_all_steps=0):
        """Grasp the target position"""
        self._execute_rl_trajectory(real_traj_path, first_n_steps, hz,
                                    hand_offset_at_n_step=hand_offset_at_n_step,
                                    hand_offset=hand_offset,
                                    hand_preoffset_for_all_steps=hand_preoffset_for_all_steps)
        pass

    def open_hand(self):
        """Release the object"""
        # open all fingers to release the object
        self.robot_comand_manager.goto_hand_joint_angles([0, 0, 0, 0, 0, 0])

    def lift(self, offset=0.1):
        """Lift the object"""
        self.robot_comand_manager.move_up(offset=offset)
        pass
    

    # def run(self):
    #     """Run the full grasp and place process"""
    #     self.goto_home()

    #     # grasp
    #     # self.goto_pregrasp()
    #     self.grasp()
    #     # self.lift()

    #     # place
    #     self.goto_preplace()
    #     self.open_hand()

    #     self.goto_home()

    #     print("Grasp and place process completed.")
