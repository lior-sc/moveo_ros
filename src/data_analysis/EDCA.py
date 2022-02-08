#!/usr/bin/env python3

import rospy
import numpy as np
from copy import deepcopy   # https://answers.ros.org/question/37682/python-deep-copy-of-ros-message/
# import timeit               # https://stackoverflow.com/questions/7370801/how-to-measure-elapsed-time-in-python 
from sensor_msgs.msg import LaserScan

### publishers 
filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=1000)
c1_pub = rospy.Publisher('/C1_scan', LaserScan, queue_size=1000)
c2_pub = rospy.Publisher('/C2_scan', LaserScan, queue_size=1000)
c3_pub = rospy.Publisher('/C3_scan', LaserScan, queue_size=1000)
c4_pub = rospy.Publisher('/C4_scan', LaserScan, queue_size=1000)
c5_pub = rospy.Publisher('/C5_scan', LaserScan, queue_size=1000)
c6_pub = rospy.Publisher('/C6_scan', LaserScan, queue_size=1000)
c7_pub = rospy.Publisher('/C7_scan', LaserScan, queue_size=1000)
c8_pub = rospy.Publisher('/C8_scan', LaserScan, queue_size=1000)
c9_pub = rospy.Publisher('/C9_scan', LaserScan, queue_size=1000)
c10_pub = rospy.Publisher('/C10_scan', LaserScan, queue_size=1000)
## cluster publisher array
cpub = [c1_pub, c2_pub, c3_pub, c4_pub, c5_pub, c6_pub, c7_pub, c8_pub, c9_pub, c10_pub]

def filter_scan(ranges, theta):
    r = np.asarray(ranges)

    max_range = 1.5 #[m]
    angle_mask = np.ones(720)
    angle_mask[range(310,720)] = 1e-6     
    angle_mask[range(0,180)] = 1e-6  

    ## conduct modifications. 
    r *= angle_mask #mask is an (1,) vector which is meant to remove irrelevant scan angles from the scan
    r = np.where(r < max_range,r,1e-6)

    zero_range_indices = np.where(r<=1e-2)[0]
    filtered_scan_indices = np.where(r>1e-2)[0]

    r = np.delete(r,zero_range_indices)
    theta = np.delete(theta,zero_range_indices)

    return [r, theta, filtered_scan_indices]

def ED_clustering(r, theta, max_distance, DEBUG=False):
    ### calculate euclidean distance between adjacent points
    theta_appended = np.zeros(theta.shape)
    d = []
    for i in range(1,np.size(r)):
        euclidean_distance = np.sqrt(r[i-1]**2+r[i]**2-2*r[i-1]*r[i]*np.cos(theta[i-1]-theta[i]))   # https://socratic.org/questions/what-is-the-formula-for-the-distance-between-two-polar-coordinates 
        d.append(euclidean_distance)
        ## need to find a faster way than a python loop. can be done via numpy
    
    ### cluster allocation     
    cluster_index = 0
    y = [cluster_index]

    for i in range(1,np.size(r)):
        if np.absolute(d[i-1]) > max_distance:
            cluster_index += 1
        y.append(cluster_index)
    
    if DEBUG==True:
        # d=np.asarray(d)
        # rospy.logdebug(y)
        rospy.loginfo(d[np.where(d>max_distance)])

    return y, d

def publish_clusters(scan, filtered_scan_indices, r, y):
    cluster_scan = deepcopy(scan)
    filtered_scan_indices = np.asarray(filtered_scan_indices)
    r = np.asarray(r)
    y = np.asarray(y)

    for i in range(len(cpub)):
        cluster_ranges = np.zeros(720)
        cluster_indices = filtered_scan_indices[np.where(y==i)]
        cluster_ranges[cluster_indices] = r[np.where(y==i)]
        cluster_scan.ranges = cluster_ranges
        cpub[i].publish(cluster_scan)
    
    return

def publish_filtered_scan(scan, r, filtered_scan_indices):
    mod_scan_ranges = np.full(len(scan.ranges), 1e-4)
    mod_scan_ranges[filtered_scan_indices] = r

    mod_scan = deepcopy(scan)
    mod_scan.ranges = mod_scan_ranges
    filtered_scan_pub.publish(mod_scan)
    
    return



def ECA_callback(scan):
    angles_array = theta=np.linspace(np.pi/2, (5/2)*np.pi, len(scan.ranges), endpoint=False)
    ### filter scan to remove unwated angles and ranges
    r, theta, filtered_scan_indices = filter_scan(scan.ranges, angles_array)
    ### retreive cluster lables
    Y,d = ED_clustering(r, theta, 20e-2, DEBUG=False)
    ### publish filtered and cluster scan topics for rviz
    publish_filtered_scan(scan, r, filtered_scan_indices)
    publish_clusters(scan, filtered_scan_indices, np.asarray(r), np.asarray(Y))
                                     
    return

if __name__ == '__main__':
    rospy.init_node('EDCA',log_level=rospy.DEBUG)
    rospy.Subscriber('/scan',LaserScan,ECA_callback)
    rospy.spin()