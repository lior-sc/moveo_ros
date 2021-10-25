#!/usr/bin/env python3
import rospy
import numpy as np

from soltrex_poc_ros.msg import soltrex_manipulator_msg
from geometry_msgs.msg import Twist

from classes.Robot_kinematics import RRRR_manipulator

class robot_teleop_subscriber:
    def __init__(self):
        # initialize robot
        link_lengths = [232.5, 235.0, 230.0, 40.0]
        self.robot = RRRR_manipulator(link_lengths)

        #initialize node, subscriber and publisher
        rospy.init_node('Robot_teleop_node',log_level=rospy.INFO)
        rospy.Subscriber('/cmd_vel',Twist,self.callback)
        self.pub = rospy.Publisher('/FML_msg',soltrex_manipulator_msg,queue_size=1)
        rospy.sleep(1)

        #set robot starting position and publish once to get there
        p0 = np.array([350,0,300,np.deg2rad(-55)])
        self.p = p0
        self.send_to_p0()

    def send_to_p0(self):
        _pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        _pub_data = Twist()
        _pub_data.linear.x = 0
        _pub_data.linear.y = 0
        _pub_data.linear.z = 0
        _pub_data.angular.z = 0
        _pub.publish(_pub_data)
        return
    
    def callback(self,msg):
        # get data from /cmd_vel
        [lx, ly, lz] = [msg.linear.x, msg.linear.y, msg.linear.z]
        [wx, wy, wz] = [msg.angular.x, msg.angular.y, msg.angular.z]

        # get & calculate location increment
        c1 = 1/1
        c2 = 1/150
        dp=np.array([c1*lx, c1*ly, c1*lz, c2*wz])
        self.p = self.p+dp
        t = self.robot.get_IK_angles(self.p)
        
        # send data to robot ndoe
        pub_data = soltrex_manipulator_msg()
        pub_data.joint_RPOS = [t[0], t[1], t[2], 0, t[3]]
        self.pub.publish(pub_data)

        # update robot class variables
        self.robot.update_robot_state(t)

        # log info
        rospy.loginfo(self.p)

if __name__ == '__main__':
    
    # this class defines the required subscribers and publishers
    robot_handle= robot_teleop_subscriber()
    rospy.spin()