#! /usr/bin/env python3

import rospy
from collections import deque
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
import sklearn.cluster

def cbScan(scan:LaserScan):
    len = scan.

if __name__ == "__main__":
    rospy.init_node("lidar_clustering")

    sub_scan = rospy.Subscriber("scan", LaserScan, cbScan)
