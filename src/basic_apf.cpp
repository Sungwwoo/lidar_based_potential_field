#include "potential_fields/basic_apf.hpp"

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

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
RCLCPP_COMPONENTS_REGISTER_NODE(potential_fields::BasicAPF)


using namespace std::chrono_literals;

namespace potential_fields{
    BasicAPF::BasicAPF(const rclcpp::NodeOptions& options): Node("ros2_basic_apf", options){
        // Get parameters from parameter server
        parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient> (this, "/apf_parameters");
        parameter_client_->wait_for_service();

        KP_ = this->get_parameter("k").as_double();
        ETA_ = this->get_parameter("eta").as_double();        
        pf_distance_ = this->get_parameter("potential_field_distance").as_double();
        linear_kp_ = this->get_parameter("linear_kp").as_double();
        angular_kp_ = this->get_parameter("angular_kp").as_double();
        angular_kd_ = this->get_parameter("angular_kd").as_double();
        acc_x_lim_ = this->get_parameter("acc_x_lim").as_double();
        vel_x_max_ = this->get_parameter("vel_x_max").as_double();
        vel_x_min_ = this->get_parameter("vel_x_min").as_double();
        acc_theta_lim_ = this->get_parameter("acc_theta_lim").as_double();
        vel_theta_max_ = this->get_parameter("vel_theta_max").as_double();
        vel_theta_min_ = this->get_parameter("vel_theta_min").as_double();
        in_place_vel_theta_ = this->get_parameter("in_place_vel_theta").as_double();
        xy_goal_tol_ = this->get_parameter("xy_goal_tolerance").as_double();
        yaw_goal_tol_ = this->get_parameter("yaw_goal_tolerance").as_double();
        check_potential_ = this->get_parameter("check_potential").as_bool();

        delete_markers_ = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_markers_.markers.push_back(marker);
        pub_markers_->publish(delete_markers_);

        f_total_ = Force();
        f_att_ = Force();
        f_rep_ = Force();


        // Set publishers and subscribers
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("potential_markers", 10);

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&BasicAPF::cb_scan, this, std::placeholders::_1)
        );

        // Set lidar parameters
        first_scan = true;
        while (!first_scan){
            RCLCPP_INFO(this->get_logger(), "Trying to set lidar parameters...");
            rclcpp::sleep_for(500ms);
        }
        RCLCPP_INFO(this->get_logger(), "Lidar parameter set");

        
        RCLCPP_INFO(this->get_logger(), "APF Initialized");

    }

    // TODO: use service
    std::vector<double> BasicAPF::getGains(){
        
    }
    std::vector<double> BasicAPF::getWeights(){
        
    }
    std::vector<double> BasicAPF::getForces(){
        
    }    
    std::vector<double> BasicAPF::getAttForce(){
        
    }
    std::vector<double> BasicAPF::getRepForce(){
        
    }
    std::vector<double> BasicAPF::getTotalForce(){
        
    }

    void BasicAPF::getGoalPose(){
        
    }
    void BasicAPF::calcAttractiveForce(){
        
    }
    void BasicAPF::calcRepulsiveForce(){
        
    }
    void BasicAPF::calcPotentialField(){
        
    }
    void BasicAPF::calcVelocity(){
        
    }
    void BasicAPF::convertFrame(){
        
    }
    void BasicAPF::cb_scan(const sensor_msgs::msg::LaserScan& scan){
        if (first_scan){
            ld_dist_max_ = scan.range_max;
            ld_dist_min_ = scan.range_min;
            ld_angle_max_ = scan.angle_max;
            ld_angle_min_ = scan.angle_min;
            ld_data_len_ = scan.ranges.size();
            ld_angle_step_ = scan.angle_increment;
            ld_link_name_ = scan.header.frame_id;
            first_scan = false;
            return;
        }

        
    }

}