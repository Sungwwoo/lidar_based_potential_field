#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros

lp = lg.LaserProjection()


def GetTF(target_frame: str, source_frame: str):

    while True:
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())

            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue


def GetRobotLocation():
    """Return the current location of robot.

    Return:
        [Location_x, Location_y] based on map frame
    """
    loc = GetTF("map", "base_link")
    robotLocation = [loc.transform.translation.x, loc.transform.translation.y]
    return robotLocation


def cbScan(scan):
    rospy.loginfo("Got Scan")
    # ranges = scan.ranges[:]
    pc2_msg = lp.projectLaser(scan)
    ox, oy, ox_viz, oy_viz = [], [], [], []
    gx, gy = [7, 7]
    rospy.loginfo("Converting to meter")

    [rx, ry] = GetRobotLocation()
    for point in pc2.read_points_list(pc2_msg):
        ox.append(point[0])
        oy.append(point[1])
        ox_viz.append(point[0])
        oy_viz.append(point[1])

    rospy.loginfo("Calculating potential field")
    pmap, minx, miny = calc_potential_field(gx - rx, gy - ry, ox, oy, resolution, rx, ry)
    data = np.array(pmap).T
    plt.figure(figsize=(8, 8))
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
    plt.grid(True)
    plt.savefig("test.png")
    plt.close()
    plt.figure(figsize=(8, 8))
    plt.ylim([-window_size / 2, window_size / 2])
    plt.xlim([0, window_size])
    plt.scatter(ox, oy, s=2, c="r")
    plt.savefig("laser.png")
    plt.close()


def calc_potential_field(gx, gy, ox, oy, reso, rx, ry):

    minx = 0
    miny = -window_size / 2
    maxx = window_size
    maxy = window_size / 2
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potentia
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx
        uo = 0
        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx - rx, gy - ry)
            uo = calc_repulsive_potential(x, y, ox, oy, window_size / 2)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    d = np.hypot(x - gx, y - gy)
    return 0.5 * KP * d


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])
    if dq <= 0.1:
        dq = 0.1

    return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2


def get_motion_model():
    # dx, dy
    motion = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]

    return motion


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


if __name__ == "__main__":
    rospy.init_node("lidar_base_potential_field", disable_signals=True)

    # tf listner
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    # Get Params
    KP = rospy.get_param("attractive_potential_gain", 5.0)
    ETA = rospy.get_param("repulsive_potential_gain", 50.0)
    ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
    ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
    ld_data_len = rospy.get_param("lidar_data_length", 716)
    window_size = rospy.get_param("window_size", 10.0)
    resolution = rospy.get_param("map_resolution", 0.5)
    robot_r = rospy.get_param("robot_radius", 5.0)

    # Subscriber
    sub_scan = rospy.Subscriber("scan", LaserScan, cbScan, queue_size=2)

    while not rospy.is_shutdown():
        rospy.spin()
