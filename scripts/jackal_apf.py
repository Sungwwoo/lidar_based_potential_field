#! /usr/bin/env python3


from lidar_based_potential_field.basic_pf import BasicAPF
import grpc
import rospy
from concurrent import futures
from robo_gym.utils import jackal_kinova_utils, apf_env_utils
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event


if __name__ == "__main__":
    rospy.init_node("pf_main")

    apf = BasicAPF()

    rospy.sleep(1)
    apf.run()
    while not rospy.is_shutdown():
        rospy.spin()
