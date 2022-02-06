#!/usr/bin/env python3

import rospy
import numpy as np
from copy import deepcopy  #https://answers.ros.org/question/37682/python-deep-copy-of-ros-message/


from sensor_msgs.msg import LaserScan

filter_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=1000)
cluster0_pub = rospy.Publisher('/C0_scan', LaserScan, queue_size=1000)
cluster1_pub = rospy.Publisher('/C1_scan', LaserScan, queue_size=1000)
cluster2_pub = rospy.Publisher('/C2_scan', LaserScan, queue_size=1000)
cluster3_pub = rospy.Publisher('/C3_scan', LaserScan, queue_size=1000)

def filter_scan(ranges, theta):
    r = np.asarray(ranges)

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
    
    ### filter scan to remove unwated angles and ranges
    r, theta, filtered_scan_indices = filter_scan(scan.ranges, angles_array)

    ### calculate euclidean distance between adjacent points
    d = []
    for i in range(1,np.size(r)):
        euclidean_distance = np.sqrt(r[i-1]**2+r[i]**2-2*r[i-1]*r[i]*np.cos(theta[i-1]-theta[i]))   # https://socratic.org/questions/what-is-the-formula-for-the-distance-between-two-polar-coordinates 
        d.append(euclidean_distance)
        ## need to find a faster way than a python loop. can be done via numpy
    
    ### cluster allocation     
    max_cluster_seperation = 0.005  #[m]
    cluster_index = 0
    cluster_allocation_vector = [cluster_index]

    for i in range(1,np.size(r)):
        if np.absolute(d[i-1])>max_cluster_seperation:
            cluster_index += 1
        cluster_allocation_vector.append(cluster_index)

    ## we still need to create scan topics for the clusters. this will be done tomorrow                                     

    return



if __name__ == '__main__':
    rospy.init_node('EDCA',log_level=rospy.INFO)
    rospy.Subscriber('/scan',LaserScan,ECA_callback)
    rospy.spin()