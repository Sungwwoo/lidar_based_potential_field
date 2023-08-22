#! /usr/bin/env python3

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
import lidar_based_potential_field.ros_utils as ros_utils
from lidar_based_potential_field.ros_utils import Point2D, Obstacle


class LidarClustering:
    def __init__(self, buffer=None):
        # tf buffer
        if buffer is None:
            self.tfBuffer = tf2_ros.Buffer()
        else:
            self.tfBuffer = buffer
            rospy.loginfo("Got tf buffer instance")
        self.tfListner = tf2_ros.TransformListener(self.tfBuffer)

        # Get node namespace
        ns = rospy.get_namespace()
        if len(ns) < 1:
            self.ns = ""
        else:
            self.ns = ns[1 : len(ns)]

        # Lidar Configuration
        try:
            # Automatically importing lidar configurations
            scan = rospy.wait_for_message("scan", LaserScan)
            self.ld_dist_max = scan.range_max
            self.ld_dist_min = scan.range_min
            self.ld_angle_max = scan.angle_max
            self.ld_angle_min = scan.angle_min
            self.ld_data_len = len(scan.ranges)
            self.ld_angle_step = scan.angle_increment
            self.ld_link_name = scan.header.frame_id
        except:
            rospy.WARN("Cannot import configurations automatically. Using manual configurations...")
            self.ld_dist_max = rospy.get_param("lidar_distance_max", 10.0)
            self.ld_dist_min = rospy.get_param("lidar_distance_min", 0.5)
            self.ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
            self.ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
            self.ld_data_len = rospy.get_param("lidar_data_length", 716)
            self.ld_angle_step = (self.ld_angle_max - self.ld_angle_min) / float(self.ld_data_len)
            self.ld_link_name = rospy.get_param("lidar_link_name", "front_laser_link")

        # Clustering Parameters
        self.n_dist_segment = rospy.get_param("n_distance_segment", 16)
        self.thresh_cluster = rospy.get_param("clustering_threshold", 2)
        self.thresh_merge = rospy.get_param("merging_threshold", 0.5)
        self.sorted_obstacles = []
        # Subscriber
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.cbScan, queue_size=2)

    def Calc_Distance_Segment(self, obs_raw: list):
        # index 0 is the closest segment
        segmented_points = [[] for i in range(0, self.n_dist_segment)]
        for i, dist in enumerate(obs_raw):
            if self.ld_dist_min <= dist < self.ld_dist_max:
                point = Point2D()
                point.Set_RTheta(dist, self.ld_angle_min + self.ld_angle_step * i)
                seg = int((dist - self.ld_dist_min) // ((self.ld_dist_max - self.ld_dist_min) / self.n_dist_segment))
                segmented_points[seg].append(point)

        return segmented_points

    def Get_Clustered_Obstacles(self, segments: list):
        # Cluster within segment
        unmerged = [[] for i in range(0, len(segments))]
        for i, segment in enumerate(segments):
            temp = []
            if len(segment) == 0:
                break
            for j in range(1, len(segment)):
                temp.append(segment[j - 1])
                if abs(segment[j].Get_Theta() - segment[j - 1].Get_Theta()) > self.thresh_cluster * self.ld_angle_step:
                    unmerged[i].append(temp)
                    temp = []
                else:
                    continue
            if len(temp) != 0:
                temp.append(segment[-1])
                unmerged[i].append(temp)

        # Merge same cluster
        obstacles = unmerged[0][:]
        for i in range(1, len(unmerged)):
            block = unmerged[i][:]
            inserted = []

            for j in range(0, len(obstacles)):
                sp1 = obstacles[j][0]
                lp1 = obstacles[j][-1]

                for k in range(0, len(block)):
                    if inserted.count(k) == 0:
                        sp2 = block[k][0]
                        lp2 = block[k][-1]

                        if ros_utils.calcDistance(sp1.rect, lp2.rect) < self.thresh_merge:
                            if sp1.polar[1] < lp2.polar[1]:
                                obstacles[j].extend(block[k])
                            else:
                                block[k].extend(obstacles[j])
                                obstacles[j] = block[k][:]
                            inserted.append(k)
                            sp1 = obstacles[j][0]
                            lp1 = obstacles[j][-1]
                        elif ros_utils.calcDistance(lp1.rect, sp2.rect) < self.thresh_merge:
                            if lp1.polar[1] < sp2.polar[1]:
                                block[k].extend(obstacles[j])
                                obstacles[j] = block[k][:]
                            else:
                                obstacles[j].extend(block[k])
                            inserted.append(k)
                            sp1 = obstacles[j][0]
                            lp1 = obstacles[j][-1]

            for j in range(0, len(block)):
                if inserted.count(j) == 0:
                    obstacles.append(block[j])

        ret = []
        for i in range(0, len(obstacles)):
            if len(obstacles[i]) >= 3:
                ret.append(Obstacle(obstacles[i]))
        ret = sorted(ret)
        self.sorted_obstacles = ret
        return ret

    def get_obstacles_list(self):
        return self.sorted_obstacles

    def cbScan(self, scan: LaserScan):
        obs_raw = scan.ranges[:]

        segmented_points = self.Calc_Distance_Segment(obs_raw)
        obstacles = self.Get_Clustered_Obstacles(segmented_points)
