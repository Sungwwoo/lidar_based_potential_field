#! /usr/bin/env python3

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
from tf2_geometry_msgs import PoseStamped
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
import lidar_based_potential_field.ros_utils as ros_utils
from lidar_based_potential_field.ros_utils import Point2D, Obstacle


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
        if ns == "/":
            self.ns = ""
        else:
            self.ns = ns[1 : len(ns)]

        # Get Params
        self.KP = rospy.get_param("attractive_potential_gain", 5.0)
        self.ETA = rospy.get_param("repulsive_potential_gain", 200.0)

        self.att_rho_o = rospy.get_param("attractive_rho_o", 1.0)

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
        self.n_dist_segment = rospy.get_param("n_distance_segment", 10)
        self.thresh_cluster = rospy.get_param("clustering_threshold", 2)
        self.thresh_merge = rospy.get_param("merging_threshold", 0.5)

        self.pf_distance = rospy.get_param("potential_field_distance", 15.0)

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

        self.is_running = False

        # Subscribere
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

        self.f_total = [0.0, 0.0]
        self.total = 0.0
        self.f_att = [0.0, 0.0]
        self.att = 0.0
        self.f_rep = [0.0, 0.0]
        self.rep = 0.0
        self.att_offset = 0.0

        # For additional initialization of inherited classes
        self.initialize()

        rospy.loginfo("APF Initialized")
        return

    def convert_frame(self, pose: list):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.ld_link_name
        pose_stamped.header.stamp = rospy.Time.now()

        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = 0
        pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1)

        try:
            converted_pose = self.tfBuffer.transform(
                pose_stamped, self.ns + "base_link", rospy.Duration(1)
            )
            return converted_pose.pose.position.x, converted_pose.pose.position.y
        except:
            rospy.logerr("Total force transform failed")
            pass
        return

    def initialize(self):
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
        loc = ros_utils.GetTF(self.tfBuffer, self.ns + "odom", self.ns + "base_link", True)
        robotLocation = [loc.transform.translation.x, loc.transform.translation.y]
        robotOrientation = [
            loc.transform.rotation.x,
            loc.transform.rotation.y,
            loc.transform.rotation.z,
            loc.transform.rotation.w,
        ]
        return robotLocation, robotOrientation

    def GetGoalPose(self):
        loc = ros_utils.GetTF(self.tfBuffer, self.ld_link_name, self.ns + "goal")

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
        marker.header.frame_id = "map"
        marker.ns = "weights"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(-1.0, 0, 0)
        marker.text = "%.2f, %.2f \n %.2f, %.2f" % (
            self.KP,
            self.ETA,
            self.prev_linear_x,
            self.prev_angular_z,
        )
        marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b = 0, 0, 0
        marker.color.a = 1
        markerArray.markers.append(marker)

        marker = Marker()
        self.f_att, self.att = self.CalcAttractiveForce(gx, gy)
        marker.header.frame_id = self.ld_link_name
        marker.ns = "attractive"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1 * self.att
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = ros_utils.calcOrientation(
            [0, 0, 0], [self.f_att[0], self.f_att[1], 0]
        )
        marker.color.r, marker.color.g, marker.color.b = 1, 0, 0
        marker.color.a = 0.7
        markerArray.markers.append(marker)

        if obstacle.polar[0] > self.ld_dist_max:
            self.f_rep, self.rep = [0, 0], 0
        else:
            self.f_rep, self.rep = self.CalcRepulsiveForce(obstacle, self.pf_distance)
            marker = Marker()
            marker.header.frame_id = self.ld_link_name
            marker.ns = "repulsive_max"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.1 * self.rep
            marker.scale.y, marker.scale.z = 0.03, 0.03
            marker.pose.position = Point(0, 0, 0)
            marker.pose.orientation = ros_utils.calcOrientation(
                [0, 0, 0], [self.f_rep[0], self.f_rep[1], 0]
            )
            marker.color.r, marker.color.g, marker.color.b = 0, 0, 1
            marker.color.a = 0.7
            markerArray.markers.append(marker)

        # Total potential
        u_total = [
            self.f_att[0] + self.f_rep[0],
            self.f_att[1] + self.f_rep[1],
        ]

        # Concerting to base_link frame
        u_total = self.convert_frame(u_total)

        u = np.hypot(u_total[0], u_total[1])

        marker = Marker()

        marker.header.frame_id = self.ns + "base_link"
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
        d = np.hypot(gx, gy)
        ratio = 1.0
        if d < self.att_rho_o:
            ratio = d / self.att_rho_o
            d = self.att_rho_o
        theta = np.arctan2(gx, gy)
        theta += self.att_offset
        p_x = self.KP * d * np.sin(theta) / ratio
        p_y = self.KP * d * np.cos(theta) / ratio
        return [p_x, p_y], self.KP * d

    def CalcRepulsiveForce(self, o, dist_max):
        [ox, oy] = o.rect
        r = o.polar[0]
        f_x = -self.ETA * (1 / r - 1 / dist_max) / r**2 * ox / r
        f_y = -self.ETA * (1 / r - 1 / dist_max) / r**2 * oy / r
        # f_x = -self.ETA * ox
        # f_y - -self.ETA * oy
        d = np.hypot(f_x, f_y)
        return [f_x, f_y], d

    def CalcVelocity(self, u_total, u, goal_orientation, relative_goal, min_dist):
        twist = Twist()
        twist.linear.y, twist.linear.z = 0, 0
        twist.angular.x, twist.angular.y = 0, 0

        current_orientation = ros_utils.calcOrientation(
            [0, 0, 0], [u_total[0], u_total[1], 0], ret_deg=True
        )

        # TODO
        if ros_utils.calcDistance([0, 0], relative_goal) < self.xy_goal_tol:
            twist.linear.x = 0.0
            if goal_orientation < -self.yaw_goal_tol:
                twist.angular.z = -self.in_place_vel_theta
            elif goal_orientation > self.yaw_goal_tol:
                twist.angular.z = self.in_place_vel_theta
            else:
                rospy.loginfo("Goal Arrived!")
                twist.angular.z = 0
            if self.check_potential:
                return
            self.pub_cmd.publish(twist)
            return

        target_ang = self.vel_theta_max * self.angular_kp * current_orientation / (
            2 * np.pi
        ) - self.angular_kd * (self.prev_orientation - current_orientation)

        # TODO: Fix linear velocity equation
        # Generate velocities
        # current_tic = rospy.Time.now()
        # dt = current_tic.to_sec() - self.prev_tic.to_sec()
        # self.prev_tic = current_tic

        # Directly using target velocities
        twist.angular.z = target_ang

        if abs(twist.angular.z) > self.vel_theta_max:
            if twist.angular.z < 0:
                twist.angular.z = -self.vel_theta_max
            else:
                twist.angular.z = self.vel_theta_max
        # target_lin = self.vel_x_max * ((1 - 1 / abs(u)) - self.linear_kp * abs(target_ang) / self.vel_theta_max)

        target_lin = self.linear_kp * u

        # Clip Velocity
        if target_lin < self.vel_x_min:
            target_lin = self.vel_x_min
        elif target_lin > self.vel_x_max:
            target_lin = self.vel_x_max

        # target_lin = min(self.vel_x_max * (1 - angular_ratio), self.vel_x_max * ((1 - 1 / abs(u))))
        angular_ratio = 1.2 * abs(target_ang) / self.vel_theta_max

        if angular_ratio > 0.9:
            angular_ratio = 0.9

        rospy.loginfo(
            "Angular Difference: {:.2f}, {:.2f}".format(
                current_orientation, self.prev_orientation - current_orientation
            )
        )

        self.prev_orientation = current_orientation

        twist.linear.x = (1 - angular_ratio) * target_lin

        # Save current velocities
        self.prev_linear_x = twist.linear.x
        self.prev_angular_z = twist.angular.z

        # Move backward if obstacle is too close
        # if min_dist < self.ld_dist_min + 0.1:
        #     twist.linear.x = -0.1

        if not self.check_potential:
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
        self.KP = KP
        self.ETA = ETA

        return

    def set_gains(self, linear_kp: float, angular_kp: float, angular_kd: float):
        self.linear_kp = linear_kp
        self.angular_kp = angular_kp
        self.angular_kd = angular_kd
        return

    def get_gains(self):
        """Get latest pd gains"""
        return [self.linear_kp, self.angular_kp, self.angular_kd]

    def get_weights(self):
        """Get positive, repulsive potential weights

        Returns:
            (positive_potential_weight, repulsive_potential_weight)
        """
        return [self.KP, self.ETA]

    def get_att_force(self):
        """Get latest attractive force

        Return:
            (f_x, f_y, |f|)
        """
        return [self.f_att[0], self.f_att[1], self.att]

    def get_rep_force(self):
        """Get latest repulsive force

        Return:
            (f_x, f_y, |f|)
        """
        return [self.f_rep[0], self.f_rep[1], self.rep]

    def get_total_force(self):
        """Get latest total force

        Return:
            (f_x, f_y, |f|)
        """
        return [self.f_total[0], self.f_total[1], self.total]

    def get_forces(self):
        ret = self.get_att_force()
        ret.extend(self.get_rep_force())
        ret.extend(self.get_total_force())
        return ret

    def set_att_offset(self, offset):
        self.att_offset = offset

    def cbScan(self, scan: LaserScan):
        # Calculate potentals only if the robot server requires
        if not self.is_running:
            return

        obs_raw = scan.ranges[:]
        min_dist_obstacle = Point2D()
        min_dist_obstacle.Set_RTheta(
            min(scan.ranges),
            self.ld_angle_min + self.ld_angle_step * obs_raw.index(min(scan.ranges)),
        )

        # [r_transx, r_transy], [r_rotx, r_roty, r_rotz, r_rotw] = GetRobotPose()
        [g_transx, g_transy], goal_orientation = self.GetGoalPose()
        self.f_total, self.total = self.CalcPotentialField(
            g_transx, g_transy, min_dist_obstacle, self.ld_dist_max
        )

        self.CalcVelocity(
            self.f_total,
            self.total,
            euler_from_quaternion(goal_orientation)[2],
            min(obs_raw),
        )


class ClusteredAPF(BasicAPF):
    def initialize(self):
        # Number of detected obstacles
        self.n_detected_obstacles = 0

        self.obst_ratio = 0.5

        # Number of obstacles for repulsive force calculation
        self.num_obstacles = 1
        return

    def Calc_Distance_Segment(self, obs_raw: list):
        # index 0 is the closest segment
        segmented_points = [[] for i in range(0, self.n_dist_segment)]
        for i, dist in enumerate(obs_raw):
            if self.ld_dist_min <= dist < self.ld_dist_max:
                point = Point2D()
                point.Set_RTheta(dist, self.ld_angle_min + self.ld_angle_step * i)
                seg = int(
                    (dist - self.ld_dist_min)
                    // ((self.ld_dist_max - self.ld_dist_min) / self.n_dist_segment)
                )
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
                if (
                    abs(segment[j].Get_Theta() - segment[j - 1].Get_Theta())
                    > self.thresh_cluster * self.ld_angle_step
                ):
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
        return ret

    def get_obstacles_list(self, scan: LaserScan):
        return self.Get_Clustered_Obstacles(self.Calc_Distance_Segment(scan.ranges))

    def CalcPotentialField(self, gx, gy, obstacles, dist_max):
        markerArray = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "weights"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(-1.0, 0, 0)
        marker.text = "%.2f, %.2f, %d" % (self.KP, self.ETA, self.num_obstacles)
        marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b = 0, 0, 0
        marker.color.a = 1
        markerArray.markers.append(marker)

        marker = Marker()
        self.f_att, self.att = self.CalcAttractiveForce(gx, gy)
        marker.header.frame_id = self.ld_link_name
        marker.ns = "attractive"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1 * self.att
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = ros_utils.calcOrientation(
            [0, 0, 0], [self.f_att[0], self.f_att[1], 0]
        )
        marker.color.r, marker.color.g, marker.color.b = 1, 0, 0
        marker.color.a = 0.7
        markerArray.markers.append(marker)

        self.f_rep, self.rep = [0, 0], 0
        if self.num_obstacles != 0 and len(obstacles) != 0:
            uo_, rep_ = [], []
            # rospy.loginfo(str(self.num_obstacles))
            # rospy.loginfo(str(len(obstacles)))

            for i in range(0, self.num_obstacles):
                # rospy.loginfo("calculating %d of %d", i + 1, self.n_detected_obstacles)
                obstacle = obstacles[i]
                uo_i, rep_i = self.CalcRepulsiveForce(obstacle.closest_point, self.pf_distance)
                uo_.append(uo_i)
                rep_.append(rep_i)
                marker = Marker()
                marker.header.frame_id = self.ld_link_name
                marker.ns = "repulsive"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 0.1 * rep_i
                marker.scale.y, marker.scale.z = 0.03, 0.03
                marker.pose.position = Point(0, 0, 0)
                marker.pose.orientation = ros_utils.calcOrientation(
                    [0, 0, 0], [uo_i[0], uo_i[1], 0]
                )
                marker.color.r, marker.color.g, marker.color.b = 1, 0, 1
                marker.color.a = 0.7
                markerArray.markers.append(marker)

            if self.num_obstacles == 1:
                self.f_rep = uo_i
            else:
                sum_rep_ = np.sum(np.array(rep_))

                for i in range(0, len(uo_)):
                    # rospy.loginfo("calculating %d of %d", i+1, len(uo_))
                    self.f_rep[0] = self.f_rep[0] + uo_[i][0] * rep_[i] / sum_rep_
                    self.f_rep[1] = self.f_rep[1] + uo_[i][1] * rep_[i] / sum_rep_
        self.rep = np.hypot(self.f_rep[0], self.f_rep[1])
        marker = Marker()
        marker.header.frame_id = self.ld_link_name
        marker.ns = "repulsive_total"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1 * self.rep
        marker.scale.y, marker.scale.z = 0.03, 0.03
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = ros_utils.calcOrientation(
            [0, 0, 0], [self.f_rep[0], self.f_rep[1], 0]
        )
        marker.color.r, marker.color.g, marker.color.b = 0, 0, 1
        marker.color.a = 0.7
        markerArray.markers.append(marker)

        # Total potential
        u_total = [
            self.f_att[0] + self.f_rep[0],
            (self.f_att[1] + self.f_rep[1]),
        ]

        # Concerting to base_link frame
        u_total = self.convert_frame(u_total)
        u = np.hypot(u_total[0], u_total[1])

        marker = Marker()

        marker.header.frame_id = self.ns + "base_link"
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

    def get_n_detected_obstacles(self):
        return self.n_detected_obstacles

    def set_num_obstacles_ratio(self, ratio):
        self.obst_ratio = ratio

    def cbScan(self, scan: LaserScan):
        if not self.is_running:
            return

        [g_transx, g_transy], goal_orientation = self.GetGoalPose()

        sorted_obstacles = self.get_obstacles_list(scan)
        self.n_detected_obstacles = len(sorted_obstacles)
        self.num_obstacles = int(np.ceil(self.n_detected_obstacles * self.obst_ratio))

        # Identical to APF if 1
        self.num_obstacles = 3

        if self.num_obstacles > self.n_detected_obstacles:
            self.num_obstacles = self.n_detected_obstacles

        self.f_total, self.total = self.CalcPotentialField(
            g_transx, g_transy, sorted_obstacles, self.ld_dist_max
        )

        self.CalcVelocity(
            self.f_total,
            self.total,
            euler_from_quaternion(goal_orientation)[2],
            [g_transx, g_transy],
            min(scan.ranges),
        )
