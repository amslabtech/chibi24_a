/**
 * @file obstacle_detector.h
 * @author Takuma Tanaka
 * @brief detection obstacles from laser scan
 * @version 0.1
 * @date 2023-08-30 
 */

#ifndef OBSTACLR_DETECTOR_A_HPP
#define OBSTACLR_DETECTOR_A_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <math.h>

class ObstacleDetector : public rclcpp::Node
{
    public:
        ObstacleDetector();
        void process();
    
    //private:
        //void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void scan_obstacle();
        bool is_ignore_scan(double angle);

        int hz_;
        int laser_step_;
        double ignore_distance_;
        std::string robot_frame_;
        std::vector<double> ignore_angle_range_list_;

        bool flag_laser_scan_ = false;

        const double PI = acos(-1); 
        double range_list_1;
        double range_list_2;
        double range_list_3;

        //ros::NodeHandle nh_;
        //std::shared_ptr<rclcpp::Node> nh_; //消した

        //ros::NodeHandle private_nh_;
        //std::shared_ptr<rclcpp::Node> private_nh_; //消した

        //ros::Subscriber laser_scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
 
        //ros::Publisher obstacle_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_pub_;  

        //geometry_msgs::msg::PoseArray obstacle_pose_array_;
        geometry_msgs::msg::PoseArray obstacle_pose_array_;

        //sensor_msgs::msg::LaserScan::SharedPtr msg laser_scan_;
        std::optional<sensor_msgs::msg::LaserScan> laser_scan_;
};

#endif // OBSTACLR_DETECTOE_H