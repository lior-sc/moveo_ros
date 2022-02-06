#!/usr/bin/env python3
import rospy
import numpy as np
from copy import deepcopy                          #https://answers.ros.org/question/37682/python-deep-copy-of-ros-message/

import pandas as pd
from matplotlib import pyplot as plt
from sensor_msgs.msg import LaserScan

filter_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=1000)
cluster0_pub = rospy.Publisher('/C0_scan', LaserScan, queue_size=1000)
cluster1_pub = rospy.Publisher('/C1_scan', LaserScan, queue_size=1000)
cluster2_pub = rospy.Publisher('/C2_scan', LaserScan, queue_size=1000)
cluster3_pub = rospy.Publisher('/C3_scan', LaserScan, queue_size=1000)

def filter_scan(ranges, angles):
    r = np.asarray(deepcopy(ranges))
    theta = deepcopy(angles)

    max_range = 1.5 #[m]
    angle_mask = np.ones(720)
    angle_mask[range(310,720)] = 1e-3     
    angle_mask[range(0,180)] = 1e-3    

    ## conduct modifications. 
    r *= angle_mask #mask is an (1,) vector which is meant to remove irrelevant scan angles from the scan
    r = np.where(r < max_range,r,1e-3)

    zero_range_indices = np.where(r<=1e-2)[0]
    filtered_scan_indices = np.where(r>1e-2)[0]

    r = np.delete(r,zero_range_indices)
    theta = np.delete(theta,zero_range_indices)

    return [r, theta, filtered_scan_indices]

def ECA_callback(scan):
    angles_array = theta=np.linspace(np.pi/2, (5/2)*np.pi, len(scan.ranges), endpoint=False)
    
    ### filter scan
    r, theta, filtered_scan_indices = filter_scan(scan.ranges, angles_array)

    ### calculate euclidean distance between adjacent points
    # Dist = 
    
    ## data preperation: convert from polar to catesian coordinate system
    X = np.transpose(np.array([r*np.sin(theta),r*np.cos(theta)]))
                                       

    return



if __name__ == '__main__':
    rospy.init_node('EDCA',log_level=rospy.INFO)
    rospy.Subscriber('/scan',LaserScan,ECA_callback)
    rospy.spin()