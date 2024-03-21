#include "local_goal_creator_a/local_goal_creator_a.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ノードの初期化
    //std::shared_ptr<LocalGoalCreator> local_goal_creator = std::make_shared<LocalGoalCreator>();
    auto  local_goal_creator = std::make_shared<LocalGoalCreator>();

    rclcpp::Rate loop_rate(local_goal_creator->get_freq()); // 制御周波数の設定
    
    while(rclcpp::ok())
    {
        local_goal_creator->process();
        rclcpp::spin_some(local_goal_creator);   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
    rclcpp::shutdown();

    return 0;
}