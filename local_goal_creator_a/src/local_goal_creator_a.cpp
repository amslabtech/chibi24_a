#include "local_goal_creator_a/local_goal_creator_a.hpp"

using namespace std::chrono_literals;

// コンストラクタ
LocalGoalCreator::LocalGoalCreator() : Node("local_goal_creator_a"), clock_(RCL_ROS_TIME)
{
    // パラメータの取得
    /*
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("index_step", index_step_);
    private_nh_.getParam("goal_index", goal_index_);
    private_nh_.getParam("target_dist_to_goal", target_dist_to_goal_);
    */

    flag_global_path_ = this->declare_parameter<bool>("flag_global_path", false);


    declare_parameter<int>("hz", 10); // ループ周期 [Hz]
    get_parameter("hz", hz_);
    declare_parameter<int>("index_step", 5); // １回で更新するインデックス数
    get_parameter("index_step", index_step_);
    declare_parameter<int>("goal_index", 50); // グローバルパス内におけるローカルゴールのインデックス
    get_parameter("goal_index", goal_index_);
    declare_parameter<double>("target_dist_to_goal", 3.0); // 現在位置-ゴール間の距離 [m]
    get_parameter("target_dist_to_goal", target_dist_to_goal_);
    
    // frame idの設定
    local_goal_.header.frame_id = "map";

    // Subscriber
    //sub_estimated_pose_ = nh_.subscribe("/estimated_pose", 1, &LocalGoalCreator::estimated_pose_callback, this);
    //sub_global_path_    = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    sub_estimated_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", rclcpp::QoS(1).reliable(),
        std::bind(&LocalGoalCreator::estimated_pose_callback, this, std::placeholders::_1));
    sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", rclcpp::QoS(1).reliable(),
        std::bind(&LocalGoalCreator::global_path_callback, this, std::placeholders::_1));

    // Publisher
    //pub_local_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
    pub_local_goal_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/local_goal", rclcpp::QoS(1).reliable());
}

// estimated_poseのコールバック関数
void LocalGoalCreator::estimated_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    //printf("estimated_pose_callback\n");
    estimated_pose_ = *msg;
}

// global_pathのコールバック関数
void LocalGoalCreator::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    //printf("global_path_callback\n");
    global_path_      = *msg;
    flag_global_path_ = true;
}

// hzを返す関数
int LocalGoalCreator::get_freq()
{
    return hz_;
}

// 唯一，main文で実行する関数
void LocalGoalCreator::process()
{
    if(flag_global_path_)
        update_goal(); // ゴールの更新

}

// ゴールの更新
void LocalGoalCreator::update_goal()
{
    double dist_to_goal = get_dist_to_goal(); // ゴールまでの距離を取得

    while(dist_to_goal < target_dist_to_goal_)
    {
        goal_index_  += index_step_;        // ゴール位置をステップ数だけ先に進める
        dist_to_goal  = get_dist_to_goal(); // ゴールまでの距離を取得

        if(goal_index_ >= global_path_.poses.size())
        {
            goal_index_ = global_path_.poses.size()-1; // グローバルゴールで固定
            break;
        }
    }

    // ゴール位置を取得し，パブリッシュ
    local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
    local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
    local_goal_.header.stamp = this->get_clock()->now(); //clock_.now();
    pub_local_goal_->publish(local_goal_);
}

// 現在位置-ゴール間の距離の取得
double LocalGoalCreator::get_dist_to_goal()
{
    const double dx = global_path_.poses[goal_index_].pose.position.x - estimated_pose_.pose.position.x;
    const double dy = global_path_.poses[goal_index_].pose.position.y - estimated_pose_.pose.position.y;

    return hypot(dx, dy);
}
