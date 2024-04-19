#ifndef BASIC_APF_HPP__
#define BASIC_APF_HPP__

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "force.hpp"
#include "point_2d.hpp"

namespace potential_fields{

    class BasicAPF : public rclcpp::Node{
        public:
            BasicAPF(const rclcpp::NodeOptions& options);

            void run(){
                is_running_ = true;
            }
            void stop(){
                is_running_ = false;
            }

            void setGains(double kp, double eta){
                KP_ = kp;
                ETA_ = eta;
            }

            std::vector<double> getGains();
            std::vector<double> getWeights();
            std::vector<double> getForces();

        private:
            void getGoalPose();
            void calcAttractiveForce();
            void calcRepulsiveForce();
            void calcPotentialField();
            void calcVelocity();
            void convertFrame();
            void cb_scan(const sensor_msgs::msg::LaserScan& scan);
            std::vector<double> getAttForce();
            std::vector<double> getRepForce();
            std::vector<double> getTotalForce();

            /************* publishers **************/ 
            // cmd_vel
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
            // visualizer
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

            /************* subscribers **************/
            // scan
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

            /************* Parameters **************/
            std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;

            // APF parameters
            double KP_, ETA_, att_rho_o_, att_offset_;
            double ld_dist_max_, ld_dist_min_, ld_angle_max_, ld_angle_min_, ld_data_len_, ld_angle_step_;
            std::string ld_link_name_;
            double pf_distance_;
            double linear_kp_, angular_kp_, angular_kd_;

            // Robot dynamics parameters
            double acc_x_lim_, vel_x_max_, vel_x_min_, acc_theta_lim_, vel_theta_max_, vel_theta_min_;
            double in_place_vel_theta_;

            // Goal tolerance
            double xy_goal_tol_, yaw_goal_tol_;

            bool check_potential_;
            bool is_running_;
            bool first_scan;
            
            visualization_msgs::msg::MarkerArray delete_markers_;
            Force f_total_, f_att_, f_rep_;
    };
}

#endif