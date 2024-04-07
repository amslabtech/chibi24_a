#include "local_path_planner_a/local_path_planner_a.hpp"

//===== メイン関数 =====
int main(int argc, char* argv[])
{
    printf("start\n");
    rclcpp::init(argc, argv); // ノードの初期化
    auto  dwa = std::make_shared<DWAPlanner>();

    printf("create_dwa\n");

    rclcpp::Rate loop_rate(dwa->get_freq()); // 制御周波数の設定    
    while(rclcpp::ok())
    {
        //printf("yobareta\n");
        dwa->process();
        rclcpp::spin_some(dwa);   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
    printf("what?????\n");
    rclcpp::shutdown();

    return 0;
}