import numpy as np
from .robot_command_manager import RobotCommandManager
import rospy 

class GraspAndPlaceExecutor:
    def __init__(self):
        # init a node here
        rospy.init_node("robot_command_manager", anonymous=True)
        
        # joint angles of the home position, order in [joint1, ..., joint7]
        ## Represent the home position of the robot
        self.home_position = np.array([-41.3, -8.7, -27.2, 74.0, 71.8, -43.3, 20.0]) /180 * np.pi
        
        self.preplace_position = [] # TODO: give a dummy place at first.
        
        # command manager
        self.robot_comand_manager = RobotCommandManager()

    def goto_home(self):
        """Return to the home position"""
        for i in range(2):
            self.robot_comand_manager.goto_joint_angles(self.home_position, [0, 0, 0, 0, 0, 0])
        print('Robot returned to home position.')
    
    def goto_pregrasp(self, target_position):
        """Optional"""
        """Speed control, softly reach the rl traj start point."""
        """Approach the target position"""
        # TODO:
        pass

    def _execute_rl_trajectory(self, real_traj_path):
        """Execute the RL trajectory"""
        
        real_traj = np.load(real_traj_path)
        self.robot_comand_manager.execute_trajectory(real_traj)
        print('Grasp trajectory executed.')

    def grasp(self, real_traj_path):
        """Grasp the target position"""
        self._execute_rl_trajectory(real_traj_path)
        pass

    def goto_preplace(self, target_position):
        """Approach the target position for placing"""
        pass

    def open_hand(self):
        """Release the object"""
        pass


    def lift(self):
        """Lift the object"""
        # TODO:
        pass

    def run(self):
        """Run the full grasp and place process"""
        self.goto_home()

        # grasp
        # self.goto_pregrasp()
        self.grasp()
        # self.lift()

        # place
        self.goto_preplace()
        self.open_hand()

        self.goto_home()

        print("Grasp and place process completed.")
