#!/usr/bin/env python3
import rospy
import numpy as np
from copy import deepcopy   #https://answers.ros.org/question/37682/python-deep-copy-of-ros-message/

from sklearn import cluster as skc
import pandas as pd
from matplotlib import pyplot as plt
from sensor_msgs.msg import LaserScan

class LSA:
    def __init__(self):
        rospy.init_node('scan_analysis_node',log_level=rospy.INFO)
        rospy.Subscriber('/scan',LaserScan,self.callback)
        self.pub = rospy.Publisher('/mod_scan',LaserScan,queue_size=1000)
        self.pub_C1 = rospy.Publisher('/Cluster_1',LaserScan,queue_size=1000)
        self.pub_C2 = rospy.Publisher('/Cluster_2',LaserScan,queue_size=1000)
        self.pub_C3 = rospy.Publisher('/Cluster_3',LaserScan,queue_size=1000)

    def callback(self,msg):
        mod_scan=self._reduce_max_distance(msg,1.50)
        P=self.convert_to_cartesian_coordinates(mod_scan)
        rospy.loginfo(msg)
        self.kmeans_clustering(msg,5)
        
        pub_data=mod_scan
        self.pub.publish(pub_data)

    
    def _find_1d_array_shape(self,array):
        msg="array length is: %d"%np.size(array)
        rospy.loginfo(msg)
    
    def _reduce_max_distance(self,scan,max_dist):
        mod_scan=deepcopy(scan)
        r=np.asarray(mod_scan.ranges)
        r=np.where(r<max_dist,r,0)
        mod_scan.ranges=r
        return mod_scan
    
    def convert_to_cartesian_coordinates(self,scan):
        r=np.asarray(deepcopy(scan.ranges))
        theta=np.linspace(0,2*np.pi,np.size(r),endpoint=False)
        p = np.array([r*np.sin(theta),r*np.cos(theta)])
        return p
    
    def kmeans_clustering(self,scan,max_clusters):
        P=self.convert_to_cartesian_coordinates(scan)
        P=P.T

        # run Kmeans clustering algo up to 5 groups and measure and store wcss    
        # distortions=[]
        # for i in range(1, max_clusters):
        #     km = skc.KMeans(n_clusters=i, 
        #         init='k-means++', 
        #         n_init=10, 
        #         max_iter=300, 
        #         random_state=0)
        #     km.fit(P)
        #     distortions.append(km.inertia_)
        
        #check elbow criteria for optimal bin detection
        # wcss=np.array(distortions)
        # wcss_nder=np.array([])
        # for i in range(1,max_clusters-1):
        #     normalized_deriv = (wcss[i]-wcss[i-1])/wcss[i]
        #     wcss_nder=np.append(wcss_nder,normalized_deriv)
        # optimal_clusters=np.argmax(np.abs(wcss_nder))+2

        # rospy.loginfo("optimal number of clusters is: %d"%optimal_clusters)
        
        optimal_clusters=3

        #retrieve optimal cluster 
        km = skc.KMeans(n_clusters=optimal_clusters, 
            init='k-means++', 
            n_init=10, 
            max_iter=300, 
            tol=1e-04,
            random_state=0)
        km.fit(P)
        y_km = km.fit_predict(P)
        
        # send different topics for all clusters    
        for i in range(0,optimal_clusters):
            #filter only dots in the cluster by zeroing range of alll other points
            result = np.where(y_km == i)
            cluster_indices = np.asarray(result[0])
            rospy.loginfo(cluster_indices)
            rospy.loginfo(scan)
            temp = np.zeros(len(scan.ranges))
            temp_ranges=np.asarray(deepcopy(scan.ranges))
            temp[cluster_indices]+=temp_ranges[cluster_indices]
            # rospy.loginfo(temp_ranges)
            pub_data=deepcopy(scan)
            pub_data.ranges=temp

            # send topics
            topic_name="/cluster_%d"%i
            cluster_pub = rospy.Publisher(topic_name,LaserScan,queue_size=1000)
            cluster_pub.publish(pub_data)
        
        # rospy.loginfo("sent clusters as topics")
        return




if __name__ == '__main__':
    laser_sub=LSA()
    rospy.spin()