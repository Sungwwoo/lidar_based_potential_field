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


class Point2D:
    def __init__(self):
        self.rect = [0, 0]
        self.polar = [0, 0]

    def Set_XY(self, x: float, y: float):
        self.rect = [x, y]
        self.polar = [np.hypot(x, y), np.arctan2(y / x)]

    def Set_RTheta(self, r: float, theta: float):
        self.rect = [r * np.cos(theta), r * np.sin(theta)]
        self.polar = [r, theta]

    def Get_XY(self):
        return self.rect

    def Get_RTheta(self):
        return self.polar

    def Get_Distance(self):
        return self.polar[0]

    def Get_Theta(self):
        return self.polar[1]


class Obstacle:
    def __init__(self):
        self.group = 0
        self.points = []

    def SetGroup(self, n: int):
        self.group = n

    def SetPoint(self, points: list):
        self.points = points

    def GetGroup(self):
        return self.group

    def GetPoints(self):
        return self.points

    def GetClosestPoint(self):
        return


def Calc_Distance_Segment(obs_raw: list):
    # index 0 is the closest segment
    # plt.figure(figsize=(11.5, 11))
    segmented_points = [[] for i in range(0, n_dist_segment)]
    for i, dist in enumerate(obs_raw):
        if ld_dist_min <= dist <= ld_dist_max:
            point = Point2D()
            point.Set_RTheta(dist, ld_angle_min + ld_angle_step * i)
            seg = int(dist // n_dist_segment)
            segmented_points[seg].append(point)
    #         plt.scatter(point.rect[0], point.rect[1], s=5, c=color[seg % len(color)])
    # plt.savefig("dist_segment.png")
    # plt.close()
    return segmented_points


def Get_Clustered_Obstacles(segments: list):
    unmerged = [[] for i in range(0, len(segments))]

    # Cluster within segment
    for i, segment in enumerate(segments):
        temp = []
        if len(segment) == 0:
            break
        for j in range(1, len(segment)):
            temp.append(segment[j - 1])
            if abs(segment[j].Get_Theta() - segment[j - 1].Get_Theta()) > 10 * ld_angle_step:
                unmerged[i].append(temp)
                temp = []
            else:
                continue
        if len(temp) != 0:
            temp.append(segment[-1])
            unmerged[i].append(temp)

    # plt.figure(figsize=(11.5, 11))
    # c = 0
    # for seg in unmerged:
    #     for i in range(0, len(seg)):
    #         c += 1
    #         for point in seg[i]:
    #             plt.scatter(point.rect[0], point.rect[1], s=5, c=color[c % len(color)])
    # plt.savefig("unmerged_obstacles.png")
    # plt.close()

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

                    if calcDistance(sp1.rect, lp2.rect) < 0.5:
                        if sp1.polar[1] < lp2.polar[1]:
                            obstacles[j].extend(block[k])
                        else:
                            block[k].extend(obstacles[j])
                            obstacles[j] = block[k][:]
                        inserted.append(k)
                        sp1 = obstacles[j][0]
                        lp1 = obstacles[j][-1]
                    elif calcDistance(lp1.rect, sp2.rect) < 0.5:
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

    plt.figure(figsize=(11.5, 11))
    for i in range(0, len(obstacles)):
        for point in obstacles[i]:
            plt.scatter(point.rect[0], point.rect[1], s=5, c=color[i % len(color)])
    plt.savefig("clustered_obstacle.png")
    plt.close()

    ret = []
    for i in range(0, len(obstacles)):
        item = Obstacle()
        item.group = i
        item.points = obstacles[i]
        ret.append(item)
    return ret


def calcDistance(a: list, b: list):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def cbScan(scan: LaserScan):

    obs_raw = scan.ranges[:]
    segmented_points = Calc_Distance_Segment(obs_raw)
    obstacles = Get_Clustered_Obstacles(segmented_points)

    ox, oy = [], []
    pc2_msg = lp.projectLaser(scan)
    points = pc2.read_points_list(pc2_msg)
    for i, point in enumerate(points):
        if scan.ranges[i] < 10.0:
            ox.append(point[0])
            oy.append(point[1])


if __name__ == "__main__":
    rospy.init_node("lidar_clustering")

    lp = lg.LaserProjection()

    # Subscriber
    sub_scan = rospy.Subscriber("scan", LaserScan, cbScan, queue_size=2)

    # Params
    ld_dist_max = rospy.get_param("lidar_distnace_max", 10.0)
    ld_dist_min = rospy.get_param("lidar_distance_min", 0.5)
    ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
    ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
    ld_data_len = rospy.get_param("lidar_data_length", 716)
    n_dist_segment = rospy.get_param("n_distance_segment", 4)
    ld_angle_step = (ld_angle_max - ld_angle_min) / float(ld_data_len)

    color = ["r", "b", "k", "g", "c", "m", "y"]
    while not rospy.is_shutdown():
        rospy.spin()
