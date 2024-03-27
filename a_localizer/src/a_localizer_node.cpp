#include "a_localizer/a_localizer.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<EMCL>();
    rclcpp::Rate loop_rate(node->getOdomFreq()); // 制御周波数の設定
    node->initialize();             // パーティクルの初期化
    
    while(rclcpp::ok())
    {
        node->process();
        rclcpp::spin_some(node);  // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }

    rclcpp::shutdown();

    return 0;
}