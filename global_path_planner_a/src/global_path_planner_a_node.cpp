#include "global_path_planner_a/global_path_planner_a.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    //Astar astar;
    //astar.process();
    
    //rclcpp::spin(std::make_shared<Astar>());
    //rclcpp::shutdown();
    


    //std::shared_ptr<Astar> astar = std::make_shared<Astar>();
    auto astar = std::make_shared<Astar>();
    rclcpp::Rate loop_rate(astar->get_freq());

    while(rclcpp::ok())
    {
        astar->process();
        rclcpp::spin_some(astar_);
        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}