"""The GraspAndPlaceExecutor class is responsible for executing the grasp and place actions."""


import numpy as np
from .robot_command_manager import RobotCommandManager
import rospy 
from scipy.spatial.transform import Rotation as R
from robot_arm_py.arm_ik import Arm_IK
import os

class GraspAndPlaceExecutor:
    def __init__(self):
        # init a node here
        rospy.init_node("robot_command_manager", anonymous=True)
        
        # joint angles of the home position, order in [joint1, ..., joint7]
        ## Represent the home position of the robot
        self.home_position_angles = np.array([-41.3, -8.7, -27.2, 74.0, 71.8, -43.3, 20.0]) /180 * np.pi
        # TODO: give a dummy place at first, replace to real position later
        self.preplace_position = np.array([-41.3, -24.9, -44.6, 77.3, 68.8, -12.7, 6.2]) /180*np.pi
        
        # command manager
        self.robot_comand_manager = RobotCommandManager()
        
        arm_right_urdf_path=f"/home/xichen/Documents/repos/Luca_Sim2Real/catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R/Realman_Inspire_R.urdf"
        arm_right_urdf_package_dirs=[
        f"/home/xichen/Documents/repos/Luca_Sim2Real/catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R"]
        self.arm_ik = Arm_IK(arm_right_urdf_path, arm_right_urdf_package_dirs)

    def goto_home(self, type='moveit', allow_moveit_fail=True, table_obstacle=None, hz=2):
        """Return to the home position"""
        # for i in range(2):
        #     self.robot_comand_manager.goto_joint_angles(self.home_position, [0, 0, 0, 0, 0, 0])
            
        rospy.sleep(2)
        home_hand_joint_angles = [0, 0, 0, 0, 0, 0]
        # self.robot_comand_manager.goto_joint_angles(self.home_position_angles, home_hand_joint_angles)
        home_eef_pose = self.arm_ik.fk(self.home_position_angles)
        # xyz = home_eef_pose[:3, 3]
        # quat = R.from_matrix(home_eef_pose[:3, :3]).as_quat()
        # xyzq = np.concatenate((xyz, quat))
        # TODO: add table obstacle to avoid collision
        try:
            self.robot_comand_manager.moveto_pose_with_moveit_plan(
                home_eef_pose, 
                home_hand_joint_angles, 
                table_obstacle=table_obstacle)
    
        except Exception as ServiceException:
            if allow_moveit_fail:
                print("Failed to reach the pregrasp position by moveit. ", ServiceException)
                print("Try to reach the pregrasp position by spliting joint angles.")

            else:
                raise ServiceException     
        home_point = np.concatenate((home_eef_pose, home_hand_joint_angles))
        self.robot_comand_manager.execute_trajectory([home_point], hz=hz)           
        
        print('Robot returned to home position.')
    
    def goto_preplace(self):
        """Approach the target position for placing"""
        rospy.sleep(2)
        # TODO: move arm but in progress, do not move the hand, -1 for not moving hand?
        self.robot_comand_manager.goto_arm_joint_angles(self.preplace_position)

    def goto_pregrasp(self, pregrasp_eef_pose_matrix, pregrasp_hand_angles, hz, 
                      type='moveit', direct_if_moveit_failed=False, table_obstacle=None):
        """Control the speed and position of pregrasp position, it matters for the grasp success"""
        """Optional"""
        """Speed control, softly reach the rl traj start point."""
        """Approach the target position"""
        matrix_to_xyzq = lambda matrix: np.concatenate([matrix[:3, 3], R.from_matrix(matrix[:3, :3]).as_quat()])
        pregrasp_eef_pose = np.array(matrix_to_xyzq(pregrasp_eef_pose_matrix))
        tscaled_first_traj_point = np.concatenate((pregrasp_eef_pose, pregrasp_hand_angles))
        
        if type != 'moveit':
            print("goto_pregrasp by spliting joint angles")
            self.robot_comand_manager.execute_trajectory([tscaled_first_traj_point], hz=hz)
            
        else:
            print("goto_pregrasp by moveit")
            # TODO: add table obstacle to avoid collision
            try:
                self.robot_comand_manager.moveto_pose_with_moveit_plan(
                    pregrasp_eef_pose, 
                    pregrasp_hand_angles, 
                    table_obstacle=None)
        
            except Exception as ServiceException:
                if direct_if_moveit_failed:
                    print("Failed to reach the pregrasp position by moveit. ", ServiceException)
                    print("Try to reach the pregrasp position by spliting joint angles.")
                    self.robot_comand_manager.execute_trajectory([tscaled_first_traj_point], hz=hz)
                else:
                    raise ServiceException                
        print('Robot is reaching the pregrasp position.')

    def _execute_rl_trajectory(self, real_traj_path, first_n_steps=120, hz=8):
        """Execute the RL trajectory"""
        
        # TODO: compute first n steps by  flag
        if first_n_steps is None:
            first_n_steps = len(real_traj)
            
        real_traj = np.load(real_traj_path)
        self.robot_comand_manager.execute_trajectory(real_traj[:first_n_steps], hz=hz)
        print('Grasp trajectory executed.')

    def grasp(self, real_traj_path, first_n_steps, hz):
        """Grasp the target position"""
        self._execute_rl_trajectory(real_traj_path, first_n_steps, hz)
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
