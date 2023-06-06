#! /usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion


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


def GetTF(tf_buffer, target_frame: str, source_frame: str):
    while True:
        try:
            trans = tf_buffer.lookup_transform(target_frame, source_frame, time=rospy.Time(0))

            return trans
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
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

    def __lt__(self, other):
        return self.closest_point.polar[0] < other.closest_point.polar[0]

    def __gt__(self, other):
        return self.closest_point.polar[0] > other.closest_point.polar[0]

    def __le__(self, other):
        return self.closest_point.polar[0] <= other.closest_point.polar[0]

    def __ge__(self, other):
        return self.closest_point.polar[0] >= other.closest_point.polar[0]

    def __eq__(self, other):
        return self.closest_point.polar == other.closest_point.polar
