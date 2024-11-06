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
        # TODO: give a dummy place at first, replace to real position later
        self.preplace_position = np.array([-41.3, -24.9, -44.6, 77.3, 68.8, -12.7, 6.2]) /180*np.pi
        
        # command manager
        self.robot_comand_manager = RobotCommandManager()

    def goto_home(self):
        """Return to the home position"""
        # for i in range(2):
        #     self.robot_comand_manager.goto_joint_angles(self.home_position, [0, 0, 0, 0, 0, 0])
            
        rospy.sleep(1)
        self.robot_comand_manager.goto_joint_angles(self.home_position, [0, 0, 0, 0, 0, 0])
        print('Robot returned to home position.')
    
    def goto_preplace(self, target_position):
        """Approach the target position for placing"""
        rospy.sleep(1)
        # TODO: move arm but in progress, do not move the hand, -1 for not moving hand?
        self.robot_comand_manager.goto_arm_joint_angles(self.preplace_position)

    def goto_pregrasp(self, real_traj_path):
        """Optional"""
        """Speed control, softly reach the rl traj start point."""
        """Approach the target position"""
        # TODO: given the target arm pose and hand angles as input
        real_traj = np.load(real_traj_path)
        self.robot_comand_manager.execute_trajectory(real_traj[:1])
        print('Robot reached the pregrasp position.')

    def _execute_rl_trajectory(self, real_traj_path):
        """Execute the RL trajectory"""
        
        real_traj = np.load(real_traj_path)
        self.robot_comand_manager.execute_trajectory(real_traj)
        print('Grasp trajectory executed.')

    def grasp(self, real_traj_path):
        """Grasp the target position"""
        self._execute_rl_trajectory(real_traj_path)
        pass

    def open_hand(self):
        """Release the object"""
        # open all fingers to release the object
        self.robot_comand_manager.goto_hand_joint_angles([0, 0, 0, 0, 0, 0])

    def lift(self, offset=0.1):
        """Lift the object"""
        self.robot_comand_manager.move_up(offset=offset)
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
