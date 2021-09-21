#!/usr/bin/env python3

import numpy as np
import sympy as sp

import rospy
from soltrex_poc_ros.msg import soltrex_manipulator_msg
from geometry_msgs.msg import Twist

class RRRR_manipulator:
    def __init__(self,l):
        self.FK = self.calculate_FK_matrix(l)
        self.IK = self.calculate_IK_vector(l)
        
        self.joint_home_position = np.array([np.pi/2, 0, 0, 0])

    def calculate_FK_matrix(self,l):
        t1,t2,t3,t4 = sp.symbols("t1, t2, t3, t4")
        
        A01 = sp.Matrix([
            [sp.cos(t1), 0, sp.sin(t1), 0],
            [sp.sin(t1), 0, -sp.cos(t1), 0],
            [0, 1, 0, l[0]],
            [0, 0, 0, 1],
            ])

        A12 = sp.Matrix([
            [sp.cos(t2), -sp.sin(t2), 0, l[1]*sp.cos(t2)],
            [sp.sin(t2), sp.cos(t2), 0,  l[1]*sp.sin(t2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])

        A23 = sp.Matrix([
            [sp.cos(t3), -sp.sin(t3), 0, l[2]*sp.cos(t3)],
            [sp.sin(t3), sp.cos(t3), 0,  l[2]*sp.sin(t3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])

        A34 = sp.Matrix([
            [sp.cos(t4), -sp.sin(t4), 0, l[3]*sp.cos(t4)],
            [sp.sin(t4), sp.cos(t4), 0,  l[3]*sp.sin(t4)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])
        
        A04 = A01*A12*A23*A34
        FK_matrix = sp.lambdify([t1, t2, t3, t4],A04,'numpy')

        return FK_matrix

    def calculate_IK_vector(self,l):
        px, py, pz, phi = sp.symbols('px, py, pz, phi')
        
        A = pz-l[0]-l[3]*sp.sin(phi)
        B = sp.sqrt(px**2+py**2)-l[3]*sp.cos(phi)
        C = (A**2+B**2+l[1]**2-l[2]**2)/(2*l[1])

        ft1 = sp.atan2(py,px)
        ft2 = 2*sp.atan2(A+sp.sqrt(A**2+B**2-C**2),B+C)     # Normally this angle has 2 possible solutions. upper and lower shoulder but we see that the lower shoulder solution does not work so we only put the upper shoulder solution. see jupyter file for referance
        ft3 = sp.asin((A-l[1]*sp.sin(ft2))/l[2])-ft2        
        ft4 = phi-ft3-ft2

        t1=sp.lambdify([px,py,pz,phi],ft1,'numpy')
        t2=sp.lambdify([px,py,pz,phi],ft2,'numpy')
        t3=sp.lambdify([px,py,pz,phi],ft3,'numpy')
        t4=sp.lambdify([px,py,pz,phi],ft4,'numpy')

        t=[t1, t2, t3, t4]
        return t
    
    def set_joint_position_deg(self, t):
        self.JPOS=np.deg2rad(t)
        return
    
    def set_cartesian_position(self, p, phi):

        try:
            t1= self.IK[0](p[0], p[1], p[2], phi)
            t2= self.IK[1](p[0], p[1], p[2], phi)
            t3= self.IK[2](p[0], p[1], p[2], phi)
            t4= self.IK[3](p[0], p[1], p[2], phi)
        except RuntimeWarning:
            rospy.logwarn("impossible value given to inverse kinematics calculation.")
            rospy.logwarn("movement not executed")
            return
        except Exception as e:
            rospy.logerr("error in IK function:"+ e)
            rospy.logerr("movement not executed")
        else:
            t=[t1, t2, t3, t4]
        return

    def set_cartesian_position(self, p):

        try:
            t1= self.IK[0](p[0], p[1], p[2], p[3])
            t2= self.IK[1](p[0], p[1], p[2], p[3])
            t3= self.IK[2](p[0], p[1], p[2], p[3])
            t4= self.IK[3](p[0], p[1], p[2], p[3])
        except RuntimeWarning:
            rospy.logwarn("impossible value given to inverse kinematics calculation.")
            rospy.logwarn("movement not executed")
            return
        except Exception as e:
            rospy.logerr("error in IK function:"+ e)
            rospy.logerr("movement not executed")
        else:
            t=[t1, t2, t3, t4]
        return t 

link_lengths = [232.5, 235.0, 230.0, 40.0]
robot=RRRR_manipulator(link_lengths)


pub = rospy.Publisher('/FML_msg',soltrex_manipulator_msg,queue_size=1)
pub_data=soltrex_manipulator_msg()
rate=rospy.Rate(40)

p0=np.array([300,0,300,np.deg2rad(-55)])

def RRRR_update_pos_callback(msg):
    [lx, ly, lz] = [msg.linear.x, msg.linear.y, msg.linear.z]
    [wx, wy, wz] = [msg.angular.x, msg.angular.y, msg.angular.z]

    C1=1/100
    C2=1/100

    dp=np.array([C1*lx, C1*ly, C1*lz, C2*wz])
    p=p0+dp
    p0=p
    
    t = robot.set_cartesian_position(p)
    pub_data.joint_RPOS=[t[0], t[1], t[2], 0, t[3]]
    pub.publish(pub_data)
    
    rate.sleep()

    

    return

def pubsub():
    rospy.init_node('Robot_teleop_node')
    rospy.Subscriber('/turtle1/cmd_vel',Twist,RRRR_update_pos_callback)
    rospy.spin()


if __name__ == '__main__':
    pubsub()