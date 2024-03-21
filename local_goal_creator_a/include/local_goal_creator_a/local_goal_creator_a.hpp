#ifndef LOCAL_GOAL_CREATOR_A_HPP
#define LOCAL_GOAL_CREATOR_A_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>


// ===== クラス =====
class LocalGoalCreator : public rclcpp::Node
{
    public:
        LocalGoalCreator(); // デフォルトコンストラクタ
        void process();
        int get_freq(); //hzを返す関数
        
    private:
        // ----- 関数 ------
        // コールバック関数
        void estimated_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

        // その他の関数
        void   update_goal();      // ゴールの更新
        double get_dist_to_goal(); // 現在位置-ゴール間の距離の取得

        // ----- 変数 -----
        int    hz_;                  // ループ周波数 [Hz]
        int    index_step_;          // １回で更新するインデックス数
        int    goal_index_;          // グローバルパス内におけるローカルゴールのインデックス
        double target_dist_to_goal_; // 現在位置-ゴール間の距離 [m]

        // msg受け取りフラグ
        bool flag_global_path_ = false;


        // ----- その他のオブジェクト -----
        // NodeHandle
        //ros::NodeHandle nh_;
        //ros::NodeHandle private_nh_;

        // Subscriber
        //ros::Subscriber sub_global_path_; 
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
        //ros::Subscriber sub_estimated_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_estimated_pose_;
        
        // Publisher
        //ros::Publisher pub_local_goal_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_local_goal_;
        
        rclcpp::Clock clock_;

        // 各種オブジェクト
        geometry_msgs::msg::PointStamped local_goal_;     // local path用の目標位置
        geometry_msgs::msg::PoseStamped  estimated_pose_; // 現在位置
        nav_msgs::msg::Path              global_path_;    // グローバルパス
};

#endif