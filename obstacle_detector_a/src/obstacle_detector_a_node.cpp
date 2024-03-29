#include "obstacle_detector_a/obstacle_detector_a.hpp"

/**
 * @brief main function 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[])
{
    //ros::init(argc, argv, "obstacle_detector");
    rclcpp::init(argc, argv);
    //auto node = rclcpp::Node::make_shared("obstacle_detector");
    std::shared_ptr<ObstacleDetector> obstacle_detector = std::make_shared<ObstacleDetector>();
    
    //ObstacleDetector obstacle_detector; //classの型全体をmain内で定義
    //obstacle_detector.process();

    rclcpp::Rate loop_rate(10); 
    while(rclcpp::ok())
    {
        //printf("----------loop start----------\n");
        if(obstacle_detector->flag_laser_scan_)
        {
            //printf("flag is true\n");
            obstacle_detector->scan_obstacle();
            //printf("piblished1\n");
            obstacle_detector->obstacle_pose_pub_->publish(obstacle_detector->obstacle_pose_array_);
            //printf("published2\n");
        }
        rclcpp::spin_some(obstacle_detector);
        loop_rate.sleep();
        //printf("----------loop end----------\n");
    }

    return 0;
}