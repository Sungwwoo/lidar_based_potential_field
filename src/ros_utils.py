#! /usr/bin/env python3
import rclpy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
from tf_transformations import quaternion_from_euler, euler_from_quaternion


# Utils
def toRad(degree: int):
    return float(degree) * np.pi / 180.0


def toDegree(rad: float):
    return rad / np.pi * 180.0


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
