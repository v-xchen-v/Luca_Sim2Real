"""
- conda activate py38
- source /opt/ros/noetic/setup.bash
- source ./catkin_ws/devel/setup.bash
- then, run this script
"""

import rospy
from ros_msra_robot.msg import ArmJointCommandMsg
import numpy as np
rospy.init_node('ready_to_call_srobot_pkg', anonymous=True)
rospy.loginfo("rospy ready")

arm_pub = rospy.Publisher('arm_robot_joint_plan_command', ArmJointCommandMsg, queue_size=1)

rate = rospy.Rate(1)
rospy.sleep(1)
# while not rospy.is_shutdown():
# for i in range(2):
arm_command = ArmJointCommandMsg()
arm_command.right_arm_data = np.array([-41.3, -8.7, -27.2, 74.0, 71.8, -43.3, 20.0]) / 180 * np.pi
# arm_command.right_arm_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.00, 0.0]
arm_command.right_ee_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm_pub.publish(arm_command)
print("arm_command published")

# rate.sleep()
    
# 'rostopic hz arm_robot_pose_direct_command', create a new terminal and run this command