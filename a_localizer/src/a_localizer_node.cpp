#include "a_localizer/a_localizer.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    // ros::init(argc, argv, "localizer"); 
    rclcpp::init(argc, argv); // ノードの初期化
    EMCL emcl;
    emcl.process();
    rclcpp::shutdown();

    return 0;
}