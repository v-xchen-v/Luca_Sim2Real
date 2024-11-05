import rospy

rospy.init_node('ready_to_call_srobot_pkg', anonymous=True)
rospy.loginfo("rospy ready")


from ros_msra_robot.msg import ArmDirectPoseCommandMsg
arm_pub = rospy.Publisher('arm_robot_pose_direct_command', ArmDirectPoseCommandMsg, queue_size=1)
arm_command = ArmDirectPoseCommandMsg()
arm_command.right_arm_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm_command.right_ee_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm_pub.publish(arm_command)
print("arm_command published")