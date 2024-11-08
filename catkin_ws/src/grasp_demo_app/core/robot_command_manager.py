import rospy
from ros_msra_robot.msg import (ArmJointCommandMsg, ArmDirectPoseCommandMsg, ArmDirectPoseDeltaCommandMsg)
from ros_msra_robot.srv import MoveOnceService, MoveOnceServiceRequest

class RobotCommandManager:
    def __init__(self) -> None:
        # Initialize the ROS publishers for different types of movement commands
        ## 7 robot dof joint angles
        ## 6 hand dof joint angles
        # self.arm_hand_joint_pub = rospy.Publisher("arm_robot_joint_direct_command", ArmJointCommandMsg, queue_size=1)
        self.arm_hand_joint_pub = rospy.Publisher("arm_robot_joint_plan_command", ArmJointCommandMsg, queue_size=1)
        
        # xyz, quaternion of arm and 6 dof of hand
        self.arm_hand_pose_pub = rospy.Publisher("arm_robot_pose_direct_command", ArmDirectPoseCommandMsg, queue_size=1)
        
        self.arm_cartisian_move_pub = rospy.Publisher(
            "arm_robot_pose_delta_direct_command", 
            ArmDirectPoseDeltaCommandMsg,
            queue_size=1
        )
        
        # a sequence of arm_hand_poses
        # self.trajectory_pub = rospy.Publisher("/???", ???, queue_size=1)

    def goto_hand_joint_angles(self, hand_joint_angles):
        """Move the robot to the specified hand joint angles"""
        
        rospy.sleep(1)
        msg = ArmJointCommandMsg()
        msg.right_ee_data = hand_joint_angles
        msg.right_arm_data = []
        self.arm_hand_joint_pub.publish(msg)
        print("command published")
        
    def goto_arm_joint_angles(self, arm_joint_angles):
        """Move the robot to the specified arm joint angles"""
        
        rospy.sleep(1)
        msg = ArmJointCommandMsg()
        msg.right_arm_data = arm_joint_angles
        msg.right_ee_data = []
        self.arm_hand_joint_pub.publish(msg)
        print("command published")
        
    def goto_joint_angles(self, arm_joint_angles, hand_joint_angles):
        """Move the robot to the specified joint angles"""
        
        rospy.sleep(1) 
        msg = ArmJointCommandMsg()
        msg.right_arm_data = arm_joint_angles
        msg.right_ee_data = hand_joint_angles
        self.arm_hand_joint_pub.publish(msg)
        print(f"go to joint angles arm: {arm_joint_angles} hand: {hand_joint_angles} command published")
        
        
    def move_up(self, offset=0.1):
        # in cartisian space, move up by offset
        msg = ArmDirectPoseDeltaCommandMsg()
        dx = offset # meter
        msg.right_arm_data = [dx, 0, 0, 0, 0, 0]
        msg.right_ee_data = []
        self.arm_cartisian_move_pub.publish(msg)
        rospy.sleep(5) # TODO:??? how to wait for executation be completed or very close to be completed??？
        print("move_up command published")
        
    def moveto_pose_with_moveit_plan(self, eef_pose, hand_joint_angles,
                                     table_obstacle=None):
        """Move the robot to the specified pose, with moveit planned trajectory"""
        x, y, z, qx, qy, qz, qw = eef_pose
        arm_command = MoveOnceServiceRequest()
        arm_command.right_arm_data = [x, y, z, qx, qy, qz, qw]
        # arm_command.right_arm_data = [x, y, z, 0, 0, 0, 1]
        arm_command.right_ee_data = hand_joint_angles # pinky, ring, middle, index, pitch, yaw
        arm_command.plan_mode = 'moveit'
        
        if table_obstacle is not None:
            arm_command.obstacle_cnt = 1
            arm_command.obstacles = []
            for i in range(arm_command.obstacle_cnt):
                arm_command.obstacles += table_obstacle
                # arm_command.obstacles += [-0.333,-0.524,0.085,-0.1638,-0.2579,0.0418,0.9513,0.3,0.3,0.001]
        task_service = rospy.ServiceProxy('move_once_service', MoveOnceService)
        response = task_service(arm_command)
        
    def moveto_pose(self, pose):
        """Move the robot to the specified pose"""
        
        msg = ArmDirectPoseCommandMsg()
        msg.right_arm_data = pose[:7]
        msg.right_ee_data = pose[7:13]
        self.arm_hand_pose_pub.publish(msg)

    def execute_trajectory(self, trajectory, hz=10):
        """Execute the specified trajectory"""
        
        rate = rospy.Rate(hz)
        
        for i, traj_point in enumerate(trajectory):
            print(f"Executing trajectory point {i+1}/{len(trajectory)}")
            x, y, z, qx, qy, qz, qw = traj_point[:7]
            hand_joints = traj_point[7:13] # pinky, ring, middle, index, pitch, yaw
            # print(f"computed hand_joints: {hand_joints}")
            arm_hand_pose_command = ArmDirectPoseCommandMsg()
            arm_hand_pose_command.right_arm_data = [x, y, z, qx, qy, qz, qw]
            arm_hand_pose_command.right_ee_data = hand_joints
            self.arm_hand_pose_pub.publish(arm_hand_pose_command)
            
            rate.sleep()
