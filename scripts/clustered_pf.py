#! /usr/bin/env python3

import rospy
from collections import deque
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros

# Utils
def toRad(degree: int):
    return float(degree) * np.pi / 180.0


def toDegree(rad: float):
    return rad / np.pi * 180.0


def calcOrientation(prev: list, current: list, ret_deg=False):
    """Calculate orientation from previous point to current point

    Args:
        current point (t)
        previous point (t - 1)"""
    # unit_vector = np.array([1, 0])
    # input_vector = np.array([point2[0] - point1[0], point2[1] - point1[1]])
    # angle = np.arccos(input_vector.dot(unit_vector) / sqrt(input_vector[0] ** 2 + input_vector[1] ** 2))

    dx = current[0] - prev[0]
    dy = current[1] - prev[1]

    # -2*PI ~ 2*PI
    if dx > 0:
        if dy > 0:  # 1
            angle = np.arctan((current[1] - prev[1]) / (current[0] - prev[0]))
        else:  # 4
            angle = np.arctan((current[1] - prev[1]) / (current[0] - prev[0]))

    elif dx < 0:
        if dy > 0:  # 2
            angle = np.pi + np.arctan((current[1] - prev[1]) / (current[0] - prev[0]))
        else:  # 3
            angle = -np.pi + np.arctan((current[1] - prev[1]) / (current[0] - prev[0]))

    else:
        if dy > 0:
            angle = toRad(90)
        elif dy < 0:
            angle = toRad(-90)
        else:
            angle = 0.0

    if ret_deg:
        return angle

    q = Quaternion()
    dat = quaternion_from_euler(0, 0, angle)
    q.x = dat[0]
    q.y = dat[1]
    q.z = dat[2]
    q.w = dat[3]
    return q


def calcDistance(a: list, b: list):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def GetTF(target_frame: str, source_frame: str):

    while True:
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, time=rospy.Time(0))

            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.01)
            continue


class Point2D:
    def __init__(self):
        self.rect = [0, 0]
        self.polar = [0, 0]

    def __str__(self):
        ret = "X: " + str(self.rect[0]) + ", Y: " + str(self.rect[1]) + "\n"
        ret = ret + "R: " + str(self.polar[0]) + ", Theta: " + str(self.polar[1]) + "\n"
        return str(ret)

    def __lt__(self, other):
        return self.polar[0] < other.polar[0]

    def __gt__(self, other):
        return self.polar[0] > other.polar[0]

    def __le__(self, other):
        return self.polar[0] <= other.polar[0]

    def __ge__(self, other):
        return self.polar[0] >= other.polar[0]

    def __eq__(self, other):
        return self.polar == other.polar

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
    def __init__(self, points=[]):
        self.points = points
        self.index = 0
        self.closest_point = min(points)

    def __iter__(self):
        return self

    def __next__(self):
        if self.index == len(self.points) - 1:
            raise StopIteration
        self.index = self.index + 1
        return self.points[self.index]


def Calc_Distance_Segment(obs_raw: list):
    # index 0 is the closest segment
    segmented_points = [[] for i in range(0, n_dist_segment)]
    for i, dist in enumerate(obs_raw):
        if ld_dist_min <= dist <= ld_dist_max:
            point = Point2D()
            point.Set_RTheta(dist, ld_angle_min + ld_angle_step * i)
            seg = int(dist // n_dist_segment)
            segmented_points[seg].append(point)

    return segmented_points


def Get_Clustered_Obstacles(segments: list):
    # Cluster within segment
    unmerged = [[] for i in range(0, len(segments))]
    for i, segment in enumerate(segments):
        temp = []
        if len(segment) == 0:
            break
        for j in range(1, len(segment)):
            temp.append(segment[j - 1])
            if abs(segment[j].Get_Theta() - segment[j - 1].Get_Theta()) > thresh_cluster * ld_angle_step:
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

                    if calcDistance(sp1.rect, lp2.rect) < thresh_merge:
                        if sp1.polar[1] < lp2.polar[1]:
                            obstacles[j].extend(block[k])
                        else:
                            block[k].extend(obstacles[j])
                            obstacles[j] = block[k][:]
                        inserted.append(k)
                        sp1 = obstacles[j][0]
                        lp1 = obstacles[j][-1]
                    elif calcDistance(lp1.rect, sp2.rect) < thresh_merge:
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
        ret.append(Obstacle(obstacles[i]))
    return ret


def GetRobotPose():
    """Return the current location of robot.

    Return:
        [Location_x, Location_y],
        [Orientation_x, Orientation_y, Orientation_z, Orientation_z]
        based on odom frame
    """
    loc = GetTF("odom", "base_link")
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


def CalcPotentialField(gx, gy, obstacles, dist_min):
    markerArray = MarkerArray()

    marker = Marker()
    ug = CalcAttractivePotential(gx, gy)
    marker.header.frame_id = "base_link"
    marker.ns = "attractive"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = ug[2]
    marker.scale.y, marker.scale.z = 0.03, 0.03
    marker.pose.position = Point(0, 0, 0)
    marker.pose.orientation = calcOrientation([0, 0, 0], [gx, gy, 0])
    marker.color.r, marker.color.g, marker.color.b = 1, 0, 0
    marker.color.a = 0.7
    markerArray.markers.append(marker)

    uo = []
    for i, obstacle in enumerate(obstacles):

        obs, rep = CalcRepulsivePotential(obstacle.closest_point, dist_min)
        uo.append(rep)

        marker = Marker()
        marker.header.frame_id = "front_laser_link"
        marker.ns = "repulsive"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = rep / 100
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(obs[0], obs[1], 0)
        marker.pose.orientation = calcOrientation([obs[0], obs[1], 0], [0, 0, 0])
        marker.color.r, marker.color.g, marker.color.b = 0, 0, 1
        marker.color.a = 0.7
        markerArray.markers.append(marker)

    pub_markers.publish(delete_marker)
    pub_markers.publish(markerArray)
    del markerArray
    return ug, uo


def CalcAttractivePotential(gx, gy):
    d = np.hypot(gx, gy)
    return [gx, gy, 0.5 * KP * d]


def CalcRepulsivePotential(o, rr):
    # calc repulsive potential
    return [o.rect, 0.5 * ETA * (1.0 / o.polar[0] - 1.0 / rr) ** 2]


def CalcAttractiveForce(gx, gy):
    p_x = KP * gx
    p_y = KP * gy
    return (p_x, p_y)


def CalcRepulsiveForce(ox, oy, dist_min):
    sum_ox, sum_oy = 0, 0
    for i in range(0, len(ox)):
        sum_ox += ox[i]
        sum_oy += oy[i]
    f_x = -ETA * (1 / ox - 1 / dist_min) / ox**2
    f_y = -ETA * (1 / oy - 1 / dist_min) / oy**2
    return (f_x, f_y)


def CalcVelocity(ug, uo, goal_orientation):
    global prev_tic
    global prev_angular_z
    global prev_linear_x
    twist = Twist()
    twist.linear.y, twist.linear.z = 0, 0
    twist.angular.x, twist.angular.y = 0, 0
    current_orientation = calcOrientation([0, 0, 0], [ug[0], ug[1], 0], ret_deg=True)
    print(goal_orientation)
    if calcDistance([0, 0], ug[:2]) < xy_goal_tol:
        twist.linear.x = 0.0
        # TODO: in place orientation check
        if goal_orientation < -yaw_goal_tol:
            twist.angular.z = -in_place_vel_theta
        elif goal_orientation > yaw_goal_tol:
            twist.angular.z = in_place_vel_theta
        else:
            twist.angular.z = 0
        pub_cmd.publish(twist)
        return

    target_x = vel_x_max * (1 - 1 / abs(ug[2]))
    target_yaw = vel_theta_max * current_orientation

    current_tic = rospy.Time.now()
    dt = current_tic.to_sec() - prev_tic.to_sec()
    prev_tic = current_tic

    if target_x > prev_linear_x:
        twist.linear.x = prev_linear_x + dt * acc_x_lim
    else:
        twist.linear.x = prev_linear_x - dt * acc_x_lim

    if target_yaw > prev_angular_z:
        twist.angular.z = prev_angular_z + dt * acc_theta_lim
    else:
        twist.angular.z = prev_angular_z - dt * acc_theta_lim

    if abs(twist.linear.x) < vel_x_min:
        twist.linear.x = vel_x_min
    elif abs(twist.linear.x) > vel_x_max:
        twist.linear.x = vel_x_max

    if abs(twist.angular.z) > vel_theta_max:
        if twist.angular.z < 0:
            twist.angular.z = -vel_theta_max
        else:
            twist.angular.z = vel_theta_max

    prev_linear_x = twist.linear.x
    prev_angular_z = twist.angular.z
    pub_cmd.publish(twist)

    return


def cbScan(scan: LaserScan):
    obs_raw = scan.ranges[:]
    segmented_points = Calc_Distance_Segment(obs_raw)
    obstacles = Get_Clustered_Obstacles(segmented_points)

    # [r_transx, r_transy], [r_rotx, r_roty, r_rotz, r_rotw] = GetRobotPose()
    [g_transx, g_transy], goal_orientation = GetGoalPose()
    ug, uo = CalcPotentialField(g_transx, g_transy, obstacles, ld_dist_min)

    CalcVelocity(ug, uo, euler_from_quaternion(goal_orientation)[2])


if __name__ == "__main__":
    rospy.init_node("lidar_base_potential_field")

    lp = lg.LaserProjection()

    # tf listner
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    # Get Params
    KP = rospy.get_param("attractive_potential_gain", 5.0)
    ETA = rospy.get_param("repulsive_potential_gain", 100.0)

    # Lidar Configuration
    ld_dist_max = rospy.get_param("lidar_distnace_max", 10.0)
    ld_dist_min = rospy.get_param("lidar_distance_min", 0.5)
    ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
    ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
    ld_data_len = rospy.get_param("lidar_data_length", 716)
    ld_angle_step = (ld_angle_max - ld_angle_min) / float(ld_data_len)

    # Clustering Parameters
    n_dist_segment = rospy.get_param("n_distance_segment", 4)
    thresh_cluster = rospy.get_param("clustering_threshold", 10)
    thresh_merge = rospy.get_param("merging_threshold", 0.5)

    # Robot dynamics parameters
    acc_x_lim = rospy.get_param("acc_x_lim", 2.5)
    vel_x_max = rospy.get_param("vel_x_max", 1.0)
    vel_x_min = rospy.get_param("vel_x_min", 0.1)
    acc_theta_lim = rospy.get_param("acc_theta_lim", 3.2)
    vel_theta_max = rospy.get_param("vel_theta_max", 1.0)
    vel_theta_min = rospy.get_param("vel_theta_min", 0.2)
    in_place_vel_theta = rospy.get_param("in_place_vel_theta", 0.2)

    # Goal Tolerances
    xy_goal_tol = rospy.get_param("xy_goal_tolerance", 0.2)
    yaw_goal_tol = rospy.get_param("yaw_goal_tolerance", 0.1)

    prev_linear_x = 0.0
    prev_angular_z = 0.0
    prev_tic = rospy.Time.now()

    # Subscriber
    sub_scan = rospy.Subscriber("scan", LaserScan, cbScan, queue_size=2)

    # Publisher
    pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    pub_markers = rospy.Publisher("markers", MarkerArray, queue_size=10)

    delete_marker = MarkerArray()
    marker = Marker()
    marker.id = 0
    marker.action = Marker.DELETEALL
    delete_marker.markers.append(marker)
    pub_markers.publish(delete_marker)

    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        exit()
