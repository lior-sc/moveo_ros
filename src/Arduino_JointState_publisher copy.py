#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node('Arduino_JointState_publisher')
pub=rospy.Publisher('/Joints',JointState,queue_size=1)
rate=rospy.Rate(10)
pub_data=JointState()
pub_data.position=[0.1, 0.2, 0.3, 0.4, 0.5]

while not rospy.is_shutdown():
    pub.publish(pub_data)
    rate.sleep()