#!/usr/bin/env python3
import rospy
import numpy as np
import copy                           #https://answers.ros.org/question/37682/python-deep-copy-of-ros-message/

from sklearn import cluster as skc
import pandas as pd
from matplotlib import pyplot as plt
from sensor_msgs.msg import LaserScan

class LSA:
    def __init__(self):
        rospy.init_node('scan_analysis_node',log_level=rospy.INFO)
        rospy.Subscriber('/scan',LaserScan,self.callback)
        self.pub = rospy.Publisher('/mod_scan',LaserScan,queue_size=1000)
        self.pub_C0 = rospy.Publisher('/Cluster_0',LaserScan,queue_size=1000)
        self.pub_C1 = rospy.Publisher('/Cluster_1',LaserScan,queue_size=1000)
        self.pub_C2 = rospy.Publisher('/Cluster_2',LaserScan,queue_size=1000)

    def callback(self,msg):
        ## filter angles and excessive ranges 
        angle_mask = np.ones(720)
        angle_mask[range(310,410)] = 0      
        filtered_scan=self._filter_scan(msg, angle_mask, 1.5)

        ## prep data and retrieve cluster map via kmeans clustering algorithm
        self.kmeans_clustering(filtered_scan)
        
        ## publish modified scan as a topic (without clustering)
        pub_data=filtered_scan
        self.pub.publish(pub_data)

    
    def kmeans_clustering(self,scan):
        ## data preperation
        r = copy.deepcopy(scan.ranges)
        zero_range_indices = np.where(r<0.1)[0]
        nonzero_range_indices = np.where(r>=0.1)[0]
        r = np.delete(r,zero_range_indices)
        P = 
        modified_dataset=self.convert_to_cartesian_coordinates(scan)
        P = modified_dataset.get("P")
        P_scan_indices = modified_dataset.get("P_scan_indices")
        P=P.T

        ## set number of clusters and conduct clustering
        N_clusters=2
        # optimal_N_clusters = self.find_optimal_kmeans_clusters(P,5)
        # N_clusters = optimal_N_clusters
        km = skc.KMeans(n_clusters=N_clusters,
                        n_init=2,
                        max_iter=300)
        km.fit(P)
        y_km = km.labels_
        scan_labels = 
        rospy.loginfo(y_km)
        
        cluster_indices = P_scan_indices [np.where(y_km == 0)[0]]
        temp = np.zeros(len(scan.ranges))
        temp[cluster_indices]=scan.ranges[cluster_indices]
        cluster_0_data = copy.deepcopy(scan)
        cluster_0_data.ranges=temp
        self.pub_C0.publish(cluster_0_data) 

        cluster_indices = P_scan_indices [np.where(y_km == 1)[0]]
        temp = np.zeros(len(scan.ranges))
        temp[cluster_indices]=scan.ranges[cluster_indices]
        cluster_1_data = copy.deepcopy(scan)
        cluster_1_data.ranges=temp
        self.pub_C1.publish(cluster_1_data) 

        return

    def _find_1d_array_shape(self, array):
        msg="array length is: %d"%np.size(array)
        rospy.loginfo(msg)


    def _filter_scan(self, scan, angle_mask, max__distance,):
        ## deep copy the LaserScan msg so that do modifications are done to original message
        mod_scan=copy.deepcopy(scan)

        ## conduct modifications. 
        r = np.asarray(mod_scan.ranges)
        r *= angle_mask                   #mask is an (1,) vector which is meant to remove irrelevant scan angles from the scan
        r = np.where(r < max__distance,r,0)

        ## make sure changes are recorded to Laserscan message and return message
        mod_scan.ranges = r
        return mod_scan


    def convert_to_cartesian_coordinates(self,r, theta):  
        theta=np.linspace(0,2*np.pi,np.size(r),endpoint=False)
        p = np.array([r*np.sin(theta),r*np.cos(theta)])
        return p
    
    def find_optimal_kmeans_clusters(self,P,cluster_search_max_value):
        # run Kmeans clustering algo up to 5 groups and measure and store wcss    
        distortions=[]
        for i in range(1, cluster_search_max_value):
            km = skc.KMeans(n_clusters=i)
            km.fit(P)
            distortions.append(km.inertia_)
        
        # check elbow criteria for optimal bin detection
        wcss=np.array(distortions)
        wcss_nder=np.array([])

        for i in range(1,cluster_search_max_value-1):
            normalized_deriv = (wcss[i]-wcss[i-1])/wcss[i]
            wcss_nder=np.append(wcss_nder,normalized_deriv)
        
        optimal_clusters=np.argmax(np.abs(wcss_nder))+2

        return optimal_clusters

        # rospy.loginfo("optimal number of clusters is: %d"%optimal_clusters)





if __name__ == '__main__':
    laser_sub=LSA()
    rospy.spin()