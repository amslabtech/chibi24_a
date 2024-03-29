#include "obstacle_detector_a/obstacle_detector_a.hpp"

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector_a")
{
    //自分らで調べたros2のyamlファイルからのパラメータの読み取り方
    //declare_parameter("hz", 0);
    //declare_parameter("laser_step", 3);
    //declare_parameter("ignore_distance", 0.01);

    hz_ = this->declare_parameter<int>("hz",10);
    laser_step_ = this->declare_parameter<int>("laser_step",3);
    ignore_distance_ = this->declare_parameter<double>("ignore_distance",0.01);

    range_list_1 = 1.5*PI/16.0;
    range_list_2 = 5.0*PI/16.0;
    range_list_3 = 10.0*PI/16.0;
    
    ignore_angle_range_list_ = this->declare_parameter<std::vector<double>>("ignore_angle_range_list",{(range_list_1),(range_list_2),(range_list_3)});
    robot_frame_ = this->declare_parameter<std::string>("base_link", "string");

    /*
    auto hz_ = this->get_parameter("hz").as_int();
    auto laser_step_ = this->get_parameter("laser_step").as_int();
    auto ignore_distance_ = this->get_parameter("ignore_distance").as_double();
    auto ignore_angle_range_list_ = this->get_parameter("ignore_angle_range_list").as_double_array();
    auto robot_frame_ = this->get_parameter("robot_frame").as_string();
    */
    

    //private_nh_.param("hz", hz_);
    /*
    hz_ = this->declare_parameter<int>("hz", 0);
    hz_ = this->get_parameter<int>("hz" , hz_);
    //おそらく上記のどちらかでうまくいくはず。launchでしっかりyamlファイルを読みこめていれば0.0が上書きされるはず？

    //private_nh_.param("laser_step", laser_step_);
    laser_step_ = this->get_parameter<int>("laser_step" , laser_step_);

    //private_nh_.param("ignore_distance", ignore_distance_);
    ignore_distance_ = this->get_parameter<double>("ignore_distance" , ignore_distance_);

    //private_nh_.param("ignore_angle_range_list", ignore_angle_range_list_);
    ignore_angle_range_list_ = this->get_parameter<double>("ignore_angle_range_list", ignore_angle_range_list_ );
    //↑わからん
    */
    
    //laser_scan_sub_ = nh_.subscribe("/scan", 1, &ObstacleDetector::laser_scan_callback, this);
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(1).reliable(), std::bind(&ObstacleDetector::laser_scan_callback, this, std::placeholders::_1));

    //obstacle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obstacle_pose", 1);
    obstacle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/local_map/obstacle", rclcpp::QoS(1).reliable());

    //robot_frame_ = this->get_parameter<char*>("robot_frame" , robot_frame_); 
    obstacle_pose_array_.header.frame_id = robot_frame_;
    //

}
/**
 * @brief main function
 */

/*
void ObstacleDetector::process()
{
    //ros::Rate loop_rate(hz_);
    rclcpp::Rate loop_rate(hz_); 

    while(rclcpp::ok())
    {
        if(flag_laser_scan_)
        {
            scan_obstacle();
            //obstacle_pose_pub_.publish(obstacle_pose_array_);
            obstacle_pose_pub_->publish(obstacle_pose_array_);
        }
        //ros::spinOnce();
        auto node = rclcpp::Node::make_shared("obstacle_detector");
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}
*/



/**
 * @brief callback function of laser scan
 * @param msg 
 */
void ObstacleDetector::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_scan_ = *msg;
    flag_laser_scan_ = true;
    //printf("work laser_scan_callback\n");
}

/**
 * @brief scan obstacle
 * 
 */
void ObstacleDetector::scan_obstacle()
{
    /*
    printf("---check parameter---\n");
    printf("hz_ = %d\n",hz_); 
    printf("laser_step_ = %d\n",laser_step_);
    printf("ignore_distance_ %lf\n",ignore_distance_);
    printf("ignore_angle_range_list_[0] = %lf\n",ignore_angle_range_list_[0]);
    printf("ignore_angle_range_list_[1] = %lf\n",ignore_angle_range_list_[1]); 
    printf("ignore_angle_range_list_[2] = %lf\n",ignore_angle_range_list_[2]);
    printf("robot_frame_ = %s\n",robot_frame_);
    */
    
    //printf("start scan_obstacle\n");
    obstacle_pose_array_.poses.clear();
    //printf("poses.clear\n");
    for(int i=0; i<laser_scan_.value().ranges.size(); i+=laser_step_)
    {
        //printf("start scan_loop\n");
        if(is_ignore_scan(laser_scan_.value().angle_min + (laser_scan_.value().angle_increment * i)))
        {
            //printf("continue\n");
            continue;
        }
        if(laser_scan_.value().ranges[i] < ignore_distance_)
        {
            //printf("start culclate\n");
            geometry_msgs::msg::Pose obs_pose;
            obs_pose.position.x = laser_scan_.value().ranges[i] * cos(laser_scan_.value().angle_min + laser_scan_.value().angle_increment * i);
            obs_pose.position.y = laser_scan_.value().ranges[i] * sin(laser_scan_.value().angle_min + laser_scan_.value().angle_increment * i);
            obstacle_pose_array_.poses.push_back(obs_pose);
            //printf("end culclate\n");
        }
    }
    //printf("end scan_obstacle\n");
}

/**
 * @brief check if the scan is ignored
 * 
 * @param angle     angle of scan
 * @return true     
 * @return false 
 */
bool ObstacleDetector::is_ignore_scan(double angle)
{
    //printf("start is_ignore_scan\n");
    for(int i = 0; i < ignore_angle_range_list_.size(); i += 2)
    {
        if((i == 0) && (ignore_angle_range_list_[i] < angle) && (angle < ignore_angle_range_list_[i + 1]))
        {
            //printf("return true (i = 0)\n");
            return true;
        }
        if((i == 2) && (ignore_angle_range_list_[i] < angle))
        {
            //printf("return true (i = 2)\n");
            return true;
        }
    }
    //printf("return false\n");
    return false;
}


