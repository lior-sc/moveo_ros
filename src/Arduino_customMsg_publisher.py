#!/usr/bin/env python

import rospy
from soltrex_poc_ros.msg import soltrex_manipulator_msg

rospy.init_node('Arduino_JointState_publisher')
pub=rospy.Publisher('/PC_custom_msg',soltrex_manipulator_msg,queue_size=1)
rate=rospy.Rate(10)
pub_data=soltrex_manipulator_msg()
pub_data.header.frame_id = "Mega2560"
pub_data.position=[0.1, 0.22, 0.333, 0.4444, 0.55555]

while not rospy.is_shutdown():
    pub.publish(pub_data)
    rate.sleep()