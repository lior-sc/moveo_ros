#!/usr/bin/env python3

import rospy
from soltrex_poc_ros.msg import soltrex_manipulator_msg

rospy.init_node('Arduino_JointState_publisher')
pub=rospy.Publisher('/FML_msg',soltrex_manipulator_msg,queue_size=1)
rate=rospy.Rate(0.25)
pub_data=soltrex_manipulator_msg()
# pub_data.joint_RPOS=[0.33, 0.33, 0.33, 0.25, 0.25]
pub_data.joint_RPOS=[0, 0, 0, 0, 0]

while not rospy.is_shutdown():
    pub_data.joint_RPOS[0] = pub_data.joint_RPOS[0]*-1
    pub_data.joint_RPOS[1] = pub_data.joint_RPOS[1]*-1
    pub_data.joint_RPOS[2] = pub_data.joint_RPOS[2]*-1
    pub_data.joint_RPOS[3] = pub_data.joint_RPOS[2]*-1
    pub_data.joint_RPOS[4] = pub_data.joint_RPOS[2]*-1
    pub.publish(pub_data)
    rate.sleep()