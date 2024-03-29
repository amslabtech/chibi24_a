/*
Ray casting 2D grid map
*/

#ifndef LOCAL_MAP_CREATOR_A_HPP
#define LOCAL_MAP_CREATOR_A_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>


// ===== クラス =====
class LocalMapCreator : public rclcpp::Node
{
public:
    LocalMapCreator(); // デフォルトコンストラクタ
    void process();
    bool get_flag_obs_poses();
    void update_map(); // マップの更新

private:
    // ----- 関数（引数あり) ------
    // コールバック関数
    void obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // その他の関数
    bool in_map(const double dist, const double angle);         // マップ内か判断
    int  get_grid_index(const double dist, const double angle); // グリッドのインデックスを返す
    int  xy_to_grid_index(const double x, const double y);      // グリッドのインデックスを返す


    // ----- 関数（引数なし）-----
    void init_map();   // マップの初期化
    

    // ----- 変数 -----
    int    hz_;       // ループ周波数 [Hz]
    double map_size_; // マップの一辺の長さ [m]
    double map_reso_; // マップの解像度 [m/cell]

    // msg受け取りフラグ
    bool flag_obs_poses_ = false;
    

    // ----- その他のオブジェクト -----
    // NodeHandle

    //ros::NodeHandle nh_;
    //std::shared_ptr<rclcpp::Node> nh_; //消した

    //ros::NodeHandle private_nh_;
    //std::shared_ptr<rclcpp::Node> private_nh_; //消した

    // Subscriber
    //ros::Subscriber sub_obs_poses_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obs_poses_;

    // Publisher
    //ros::Publisher pub_local_map_;
    //rclcpp::Publisher<nav_msgs:msg::OccupancyGrid>::SharedPtr pub_local_map_;   
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_map_;

    // 各種オブジェクト
    geometry_msgs::msg::PoseArray obs_poses_; // 障害物のポーズ配列
    nav_msgs::msg::OccupancyGrid  local_map_; // ローカルマップ
};

#endif