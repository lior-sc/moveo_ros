#!/usr/bin/env python3

from numpy.lib.function_base import gradient
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from sensor_msgs.msg import LaserScan


class LaserScan_Subscriber:
    def __init__(self) -> None:
        self.vscan=np.zeros([4,720])

        rospy.init_node('LaserScan_manipulation_node',log_level=rospy.INFO)
        rospy.Subscriber('/scan',LaserScan,self.callback)

        
        
        plt.tight_layout()
        plt.show()

        rospy.spin()
        
        pass

    
    def callback(self,msg):
        angles=np.linspace(0,4*np.pi,720)
        ranges=np.array(msg.ranges)
        intensities = np.array(msg.intensities)
        grad=np.gradient(ranges,(4*np.pi)/720)
        
        self.vscan=np.matrix([angles,ranges,intensities,grad])

        rospy.loginfo(np.shape(self.vscan))
        FuncAnimation(plt.gcf(), self.animation)




    def animation(self):
        plt.plot(self.vscan[0,:],self.vscan[1,:])
        


if __name__ == '__main__':
    scan = LaserScan_Subscriber()