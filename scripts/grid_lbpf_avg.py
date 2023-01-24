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

lp = lg.LaserProjection()


def GetTF(target_frame: str, source_frame: str):

    while True:
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, time=rospy.Time(0))

            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue


def GetRobotPose():
    """Return the current location of robot.

    Return:
        [Location_x, Location_y],
        [Orientation_x, Orientation_y, Orientation_z, Orientation_z]
        based on map frame
    """
    loc = GetTF("map", "base_link")
    robotLocation = [loc.transform.translation.x, loc.transform.translation.y]
    robotOrientation = [
        loc.transform.rotation.x,
        loc.transform.rotation.y,
        loc.transform.rotation.z,
        loc.transform.rotation.w,
    ]
    return robotLocation, robotOrientation


def GetGoalPose():

    loc = GetTF("base_link", "goal")
    goalLocation = [loc.transform.translation.x, loc.transform.translation.y]
    goalOrientation = [
        loc.transform.rotation.x,
        loc.transform.rotation.y,
        loc.transform.rotation.z,
        loc.transform.rotation.w,
    ]

    return goalLocation, goalOrientation


def cbScan(scan):
    # rospy.loginfo("Got Scan")
    # ranges = scan.ranges[:]
    pc2_msg = lp.projectLaser(scan)
    ox, oy, ox_viz, oy_viz = [], [], [], []
    gx, gy = [7, 7]
    # rospy.loginfo("Converting to meter")

    [r_transx, r_transy], [r_rotx, r_roty, r_rotz, r_rotw] = GetRobotPose()
    [g_transx, g_transy], [g_rotx, g_roty, g_rotz, g_rotw] = GetGoalPose()

    if np.hypot(g_transx, g_transy) < goal_tolerance:
        return

    print([g_transx, g_transy])

    points = pc2.read_points_list(pc2_msg)
    for i, point in enumerate(points):
        if scan.ranges[i] < window_size * np.sqrt(2):
            ox.append(point[0])
            oy.append(point[1])
            ox_viz.append(point[0])
            oy_viz.append(point[1])

    # rospy.loginfo("Calculating potential field")
    pmap, minx, miny = CalcPotentialField(g_transx, g_transy, ox, oy, resolution, r_transx, r_transy)
    data = np.array(pmap).T

    # Find Local Path
    previous_ids = deque()
    px, py = [], []
    minp = float("inf")
    minix, miniy = -1, -1
    ix, iy = window_size / 8 / resolution, window_size / 2 / resolution
    px.append(ix)
    py.append(iy)
    inx, iny = ix, iy

    while True:
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                break
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        px.append(ix)
        py.append(iy)

        if oscillations_detection(previous_ids, ix, iy):
            break

    plt.figure(figsize=(8, 7))
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
    plt.plot(px, py, "r")
    plt.grid(True)
    plt.savefig("test.png")
    plt.close()


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def CalcPotentialField(gx, gy, ox, oy, reso, rx, ry):

    minx = -window_size / 8
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
            ug = CalcAttractivePotential(x, y, gx, gy)
            uo, uo_avg = CalcRepulsivePotential(x, y, ox, oy, window_size / 2)
            uf = ug + uo_avg
            pmap[ix][iy] = uf

    return pmap, minx, miny


def CalcAttractivePotential(x, y, gx, gy):
    d = np.hypot(x - gx, y - gy)
    return 0.5 * KP * d


def CalcRepulsivePotential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    sum = 0
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i
        if d <= 0.1:
            d = 0.1
        sum += 0.5 * ETA * (1.0 / d - 1.0 / rr)

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])
    if dq <= 0.1:
        dq = 0.1

    return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2, sum / len(ox)


if __name__ == "__main__":
    rospy.init_node("lidar_base_potential_field", disable_signals=True)

    # tf listner
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    # Get Params
    KP = rospy.get_param("attractive_potential_gain", 10.0)
    ETA = rospy.get_param("repulsive_potential_gain", 200.0)
    ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
    ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
    ld_data_len = rospy.get_param("lidar_data_length", 716)
    window_size = rospy.get_param("window_size", 10.0)
    resolution = rospy.get_param("map_resolution", 0.5)
    robot_r = rospy.get_param("robot_radius", 5.0)
    goal_tolerance = rospy.get_param("xy_goal_tolerance", 0.2)
    OSCILLATIONS_DETECTION_LENGTH = rospy.get_param("oscillation_detection_length", 3)
    # Subscriber
    sub_scan = rospy.Subscriber("scan", LaserScan, cbScan, queue_size=2)

    # Motions for path planning
    motion = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        exit()
