#!/usr/bin/env python3

import rospy
from soltrex_poc_ros.msg import soltrex_manipulator_msg

rospy.init_node('Arduino_JointState_publisher')
pub=rospy.Publisher('/PC_custom_msg',soltrex_manipulator_msg,queue_size=1)
rate=rospy.Rate(0.5)
pub_data=soltrex_manipulator_msg()
pub_data.joint_RPOS=[0, 0, 0, 0, 0]
pub.publish(pub_data)

while not rospy.is_shutdown():
    # pub.publish(pub_data)
    rate.sleep()