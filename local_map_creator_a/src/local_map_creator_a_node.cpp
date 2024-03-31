#include "local_map_creator_a/local_map_creator_a.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    //ros::init(argc, argv, "local_map_creator"); // ノードの初期化
    rclcpp::init(argc, argv);
    //auto node = rclcpp::Node::make_shared("local_map_creator");
    std::shared_ptr<LocalMapCreator> local_map_creator_ = std::make_shared<LocalMapCreator>();

    //LocalMapCreator local_map_creator;


    //local_map_creator.process();
    
    rclcpp::Rate loop_rate(10); // 制御周波数の設定
    while(rclcpp::ok())
    {
        //printf("-----loop start -----\n");
        if(local_map_creator_->get_flag_obs_poses())
        {
            //printf("update_map\n");
            local_map_creator_->update_map();  // マップの更新
        }
            
        //ros::spinOnce();   // コールバック関数の実行
        rclcpp::spin_some(local_map_creator_);
        loop_rate.sleep(); // 周期が終わるまで待つ
        //printf("-----loop end -----\n"); 
    }

    rclcpp::shutdown(); //追加した（ないとctrl+cでノードが止まらない）
    return 0;
}