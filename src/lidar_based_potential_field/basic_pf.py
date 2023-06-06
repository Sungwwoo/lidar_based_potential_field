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
from lidar_based_potential_field.ros_utils import Point2D


class BasicAPF:
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
        self.ns = ns[1 : len(ns) - 1]

        # Get Params
        self.KP = rospy.get_param("attractive_potential_gain", 5.0)
        self.ETA = rospy.get_param("repulsive_potential_gain", 200.0)

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
            rospy.loginfo("Successfully exctracted informations from scan topic")

        except:
            rospy.logwarn("Cannot import configurations automatically. Using manual configurations...")
            self.ld_dist_max = rospy.get_param("lidar_distance_max", 10.0)
            self.ld_dist_min = rospy.get_param("lidar_distance_min", 0.5)
            self.ld_angle_max = rospy.get_param("lidar_angle_max", 1.57619449019)
            self.ld_angle_min = rospy.get_param("lidar_angle_min", -1.57619449019)
            self.ld_data_len = rospy.get_param("lidar_data_length", 716)
            self.ld_angle_step = (self.ld_angle_max - self.ld_angle_min) / float(self.ld_data_len)
            self.ld_link_name = rospy.get_param("lidar_link_name", "front_laser_link")

        # Clustering Parameters
        self.n_dist_segment = rospy.get_param("n_distance_segment", 4)
        self.thresh_cluster = rospy.get_param("clustering_threshold", 10)
        self.thresh_merge = rospy.get_param("merging_threshold", 0.5)

        self.pf_distance = rospy.get_param("potential_field_distance")

        # Robot dynamics parameters
        self.acc_x_lim = rospy.get_param("acc_x_lim", 2.5)
        self.vel_x_max = rospy.get_param("vel_x_max", 1.0)
        self.vel_x_min = rospy.get_param("vel_x_min", 0.1)
        self.acc_theta_lim = rospy.get_param("acc_theta_lim", 3.2)
        self.vel_theta_max = rospy.get_param("vel_theta_max", 1.0)
        self.vel_theta_min = rospy.get_param("vel_theta_min", 0.2)
        self.in_place_vel_theta = rospy.get_param("in_place_vel_theta", 0.2)
        self.linear_kp = rospy.get_param("linear_kp")
        self.angular_kp = rospy.get_param("angular_kp")
        self.angular_kd = rospy.get_param("angular_kd")

        # Goal Tolerances
        self.xy_goal_tol = rospy.get_param("xy_goal_tolerance", 0.2)
        self.yaw_goal_tol = rospy.get_param("yaw_goal_tolerance", 0.1)

        self.check_potential = rospy.get_param("check_potential", False)
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.prev_tic = rospy.Time.now()
        self.prev_orientation = 0.0

        # Subscriber
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.cbScan, queue_size=2)

        # Publisher
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_markers = rospy.Publisher("potential_markers", MarkerArray, queue_size=10)

        self.delete_marker = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.delete_marker.markers.append(marker)
        self.pub_markers.publish(self.delete_marker)

        self.u_total, self.u = 0, 0

        self.is_running = False
        rospy.loginfo("BasicAPF Initialized")
        return

    def run(self):
        rospy.loginfo("Enabling APF")
        self.is_running = True

    def stop(self):
        rospy.loginfo("Disabling APF")
        self.is_running = False

    def GetRobotPose(self):
        """Return the current location of robot.

        Return:
            [Location_x, Location_y],
            [Orientation_x, Orientation_y, Orientation_z, Orientation_z]
            based on odom frame
        """
        loc = ros_utils.GetTF(self.tfBuffer, self.ns + "/odom", self.ns + "/base_link")
        robotLocation = [loc.transform.translation.x, loc.transform.translation.y]
        robotOrientation = [
            loc.transform.rotation.x,
            loc.transform.rotation.y,
            loc.transform.rotation.z,
            loc.transform.rotation.w,
        ]
        return robotLocation, robotOrientation

    def GetGoalPose(self):
        loc = ros_utils.GetTF(self.tfBuffer, self.ld_link_name, self.ns + "/goal")
        goalLocation = [loc.transform.translation.x, loc.transform.translation.y]
        goalOrientation = [
            loc.transform.rotation.x,
            loc.transform.rotation.y,
            loc.transform.rotation.z,
            loc.transform.rotation.w,
        ]

        return goalLocation, goalOrientation

    def CalcPotentialField(self, gx, gy, obstacle, dist_max):
        markerArray = MarkerArray()

        marker = Marker()
        ug, att = self.CalcAttractiveForce(gx, gy)
        marker.header.frame_id = self.ld_link_name
        marker.ns = "attractive"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1 * att
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = ros_utils.calcOrientation([0, 0, 0], [gx, gy, 0])
        marker.color.r, marker.color.g, marker.color.b = 1, 0, 0
        marker.color.a = 0.7
        markerArray.markers.append(marker)

        if obstacle.polar[0] > self.ld_dist_max:
            uo, rep = [0, 0], 0
        else:
            uo, rep = self.CalcRepulsiveForce(obstacle, self.pf_distance)
            marker = Marker()
            marker.header.frame_id = self.ld_link_name
            marker.ns = "repulsive_max"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.1 * rep
            marker.scale.y, marker.scale.z = 0.03, 0.03
            marker.pose.position = Point(0, 0, 0)
            marker.pose.orientation = ros_utils.calcOrientation([0, 0, 0], [uo[0], uo[1], 0])
            marker.color.r, marker.color.g, marker.color.b = 0, 0, 1
            marker.color.a = 0.7
            markerArray.markers.append(marker)

        # Total potential
        u_total = [ug[0] + uo[0], ug[1] + uo[1]]
        u = np.hypot(u_total[0], u_total[1])

        marker = Marker()

        marker.header.frame_id = self.ld_link_name
        marker.ns = "total"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1 * u
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = ros_utils.calcOrientation([0, 0, 0], [u_total[0], u_total[1], 0])
        marker.color.r, marker.color.g, marker.color.b = 0, 1, 0
        marker.color.a = 0.7
        markerArray.markers.append(marker)

        self.pub_markers.publish(self.delete_marker)
        self.pub_markers.publish(markerArray)
        del markerArray

        return u_total, u

    def CalcAttractivePotential(self, gx, gy):
        d = 0.5 * self.KP * np.hypot(gx, gy)
        theta = ros_utils.calcOrientation([0, 0], [gx, gy], ret_deg=True)
        dir = [d * np.cos(theta), d * np.sin(theta)]
        return dir, d

    def CalcRepulsivePotential(self, o, rr):
        # calc repulsive potential
        d = 0.5 * self.ETA * (1.0 / o.polar[0] - 1.0 / rr) ** 2

        return [d * np.cos(o.polar[1]), d * np.sin(o.polar[1])], d

    def CalcAttractiveForce(self, gx, gy):
        p_x = self.KP * gx
        p_y = self.KP * gy
        return [p_x, p_y], self.KP * np.hypot(gx, gy)

    def CalcRepulsiveForce(self, o, dist_max):
        [ox, oy] = o.rect
        r = o.polar[0]
        f_x = -self.ETA * (1 / r - 1 / dist_max) / r**2 * ox / r
        f_y = -self.ETA * (1 / r - 1 / dist_max) / r**2 * oy / r
        return [f_x, f_y], np.hypot(f_x, f_y)

    def CalcVelocity(self, u_total, u, goal_orientation, min_dist):
        twist = Twist()
        twist.linear.y, twist.linear.z = 0, 0
        twist.angular.x, twist.angular.y = 0, 0

        current_orientation = ros_utils.calcOrientation([0, 0, 0], [u_total[0], u_total[1], 0], ret_deg=True)

        # TODO: find minimum potential
        if ros_utils.calcDistance([0, 0], u_total) < self.xy_goal_tol:
            twist.linear.x = 0.0
            if goal_orientation < -self.yaw_goal_tol:
                twist.angular.z = -self.in_place_vel_theta
            elif goal_orientation > self.yaw_goal_tol:
                twist.angular.z = self.in_place_vel_theta
            else:
                twist.angular.z = 0
            if self.check_potential:
                return
            self.pub_cmd.publish(twist)
            return

        # TODO: fix velocity equation, add differential parameter
        target_x = self.vel_x_max * (1 - 1 / abs(u))
        target_yaw = self.vel_theta_max * 7 * current_orientation / (2 * np.pi) - 4 * (self.prev_orientation - current_orientation)
        self.prev_orientation = current_orientation

        # Generate velocities
        current_tic = rospy.Time.now()
        dt = current_tic.to_sec() - self.prev_tic.to_sec()
        self.prev_tic = current_tic

        if target_x > self.prev_linear_x:
            twist.linear.x = self.prev_linear_x + dt * self.acc_x_lim
        else:
            twist.linear.x = self.prev_linear_x - dt * self.acc_x_lim

        if target_yaw > self.prev_angular_z:
            twist.angular.z = self.prev_angular_z + dt * self.acc_theta_lim
        else:
            twist.angular.z = self.prev_angular_z - dt * self.acc_theta_lim

        if abs(twist.linear.x) < self.vel_x_min:
            twist.linear.x = self.vel_x_min
        elif abs(twist.linear.x) > self.vel_x_max:
            twist.linear.x = self.vel_x_max

        if abs(twist.angular.z) > self.vel_theta_max:
            if twist.angular.z < 0:
                twist.angular.z = -self.vel_theta_max
            else:
                twist.angular.z = self.vel_theta_max

        # Save current velocities
        self.prev_linear_x = twist.linear.x
        self.prev_angular_z = twist.angular.z

        # Move backward if obstacle is too close
        if min_dist < self.ld_dist_min + 0.1:
            twist.linear.x = -0.1

        if self.check_potential:
            return
        self.pub_cmd.publish(twist)

        return

    def set_weights(self, KP: float, ETA: float):
        """Set positive, repulsive potential weights

        Args:
            KP: positive potential weight
            ETA: Repulsive potential weight

        Return:
            None

        """
        self.ETA = ETA
        self.KP = KP

    def get_weights(self):
        """Get positive, repulsive potential weights

        Returns:
            [ positive_potential_weight, repulsive_potential_weight ]
        """
        return [self.KP, self.ETA]

    def get_forces(self):
        """Get latest total force

        Return:
            [ f_x, f_y, |f| ]
        """
        return [self.u_total[0], self.u_total[1], self.u]

    def cbScan(self, scan: LaserScan):
        # Calculate potentals only if the robot server requires
        if not self.is_running:
            return

        obs_raw = scan.ranges[:]
        min_dist_obstacle = Point2D()
        min_dist_obstacle.Set_RTheta(min(scan.ranges), self.ld_angle_min + self.ld_angle_step * obs_raw.index(min(scan.ranges)))

        # [r_transx, r_transy], [r_rotx, r_roty, r_rotz, r_rotw] = GetRobotPose()
        [g_transx, g_transy], goal_orientation = self.GetGoalPose()
        self.u_total, self.u = self.CalcPotentialField(g_transx, g_transy, min_dist_obstacle, self.ld_dist_max)

        self.CalcVelocity(self.u_total, self.u, euler_from_quaternion(goal_orientation)[2], min(obs_raw))
