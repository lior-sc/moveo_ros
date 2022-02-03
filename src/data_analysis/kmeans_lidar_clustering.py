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

        self.delta_theta = 2 * np.pi / 720
        self.theta_0 = np.pi/2
        self.theta_719 = self.theta_0 + 2*np.pi - self.delta_theta

    def callback(self,msg):
        ## methods 
        def construct_cluster_scan(scan, cluster_indices):
            cluster_scan = copy.deepcopy(scan)
            cluster_ranges = np.zeros(720)
            cluster_ranges[cluster_indices] = filtered_scan.ranges[cluster_indices]
            cluster_scan.ranges = cluster_ranges 
            return cluster_scan

        ## filter angles and excessive ranges 
        angle_mask = np.ones(720)
        angle_mask[range(310,720)] = 0.01     
        angle_mask[range(0,180)] = 0.01    
        filtered_scan=self._filter_scan(msg, angle_mask, 1.5)

        ## prep data and retrieve cluster map via kmeans clustering algorithm
        clusters = self.kmeans_clustering(filtered_scan)
          
        ## publish clusters and filtered scan as separate scans
        cluster_N0_scan = construct_cluster_scan(filtered_scan,clusters[0])
        cluster_N1_scan = construct_cluster_scan(filtered_scan,clusters[1])
        # cluster_N2_scan = construct_cluster_scan(filtered_scan,clusters[2])
        
        self.pub.publish(filtered_scan)
        self.pub_C0.publish(cluster_N0_scan)
        self.pub_C1.publish(cluster_N1_scan)
        # self.pub_C2.publish(cluster_N2_scan)
        
        return 

    def kmeans_clustering(self,scan):
        ### this function will cluster the laser scan using the Kmeans method which 
        ### is an unsupervised machine learning algorithm 
        
        ## internal method definitions
        def find_optimal_kmeans_clusters(X,cluster_search_max_value):
            ### this algorithm will run kmeans for N times in order to find the cluster number in which 
            ### The normalized change in WCSS is most significant. WCSS (within cluster sum of squares) 
            ### is the sum of squared distance between each point and the centroid in a cluster.

            ## run Kmeans clustering algo up to N groups and measure and store wcss   
            distortions=[]
            for i in range(1, cluster_search_max_value):
                km = skc.KMeans(n_clusters=i)
                km.fit(X)
                distortions.append(km.inertia_)
            
            ## check elbow criteria for optimal bin detection
            wcss=np.array(distortions)
            wcss_nder=np.array([])

            for i in range(1,cluster_search_max_value-1):
                normalized_deriv = (wcss[i]-wcss[i-1])/wcss[i]
                wcss_nder=np.append(wcss_nder,normalized_deriv)
            
            optimal_clusters=np.argmax(np.abs(wcss_nder))+2

            return optimal_clusters

        def organize_clusters(km):
            y_km = km.labels_
            y_cc = km.cluster_centers_
            
            N_clusters = np.shape(y_cc)[0]
            new_cluster_order = np.argsort(y_cc[:,1])[::-1]  # from high to low according to y value
            
            y_new = copy.deepcopy(y_km)+1000
            ycc_new = y_cc[new_cluster_order[:],:]

            for i in range(N_clusters ):
                y_new[np.where(y_new == 1000+i)] = np.where(new_cluster_order==i)[0]   

            # rospy.loginfo(y_cc[new_cluster_order[:],1])

            return [y_new, ycc_new]


        ## data preperation: remove points with range=0 from dataset
        r = np.asarray(copy.deepcopy(scan.ranges))
        theta=np.linspace(np.pi/2,
                          (5/2)*np.pi,
                          np.size(r),
                          endpoint=False)

        zero_range_indices = np.where(r<0.1)[0]
        nonzero_range_indices = np.where(r>=0.1)[0]

        r = np.delete(r,zero_range_indices)
        theta = np.delete(theta,zero_range_indices)
        
        ## data preperation: convert from polar to catesian coordinate system
        X = np.transpose(np.array([r*np.sin(theta),r*np.cos(theta)]))

        ## set number of clusters and conduct clustering
        # optimal_N_clusters = find_optimal_kmeans_clusters(X,5)
        # N_clusters = optimal_N_clusters
        N_clusters = 2
        km = skc.KMeans(n_clusters=N_clusters,
                        n_init=2,
                        max_iter=300)
        km.fit(X)

        # Y = organize_clusters(km)
        [y_km, y_cc] = organize_clusters(km)

        # rospy.loginfo(y_cc[:,1])
        
        cluster_scan_indices = []
        for i in range(0,N_clusters):
            cluster_scan_indices.append(
                nonzero_range_indices[np.where(y_km == i)[0]])

        return cluster_scan_indices

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



if __name__ == '__main__':
    laser_sub=LSA()
    rospy.spin()