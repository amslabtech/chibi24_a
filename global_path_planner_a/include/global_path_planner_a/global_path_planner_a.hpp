#ifndef GLOBAL_PATH_PLANNER_A_HPP
#define GLOBAL_PATH_PLANNER_A_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <math.h>


//　構造体
struct Node_
{
    int    x = 0;
    int    y = 0;
    int    parent_x = -1;
    int    parent_y = -1;
    double f = 0.0;
};

struct Motion_
{
    int dx;
    int dy;
    double cost;
};

class Astar : public rclcpp::Node
{
    public:
        Astar();

        void process();
        int get_freq();

    private:
        void planning();  //経路計画

        void timer_callback();
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  //マップの読み込み


        void obs_expander(); // obstacle expander
        void obs_expand(const int index); //障害物の拡張
        void swap_node(const Node_ node, std::vector<Node_>& list1,std::vector<Node_>& list2);  //list1からlist2にノードを移動
        // void get_way_points(std::vector<std::vector<int>>& list);  //経由点の宣言
        void create_path(Node_ node);  //waypoint間のパスを作成し、グローバルパスに追加
        void update_list(const Node_ node);  //隣接ノードを基にopenリスト・closeリストを更新
        void create_neighbor_nodes(const Node_ node, std::vector<Node_>& nodes);  //現在のノードを元に隣接ノードを作成
        void get_motion(std::vector<Motion_>& motion);  //動作モデルを定義
        
        void show_node_point(const Node_ node);  //［デバック用］ノードをRviz
        void show_path(nav_msgs::msg::Path& current_path);  //［デバック用］パスをRviz
        void show_exe_time(); //実行時間を表示（スタート時間beginをあらかじめ設定する）

        bool check_start(const Node_ node);  //スタートノードの場合、trueを返す
        bool check_goal(const Node_ node);  //ゴールノードの場合、trueを返す
        bool check_same_node(const Node_ n1, const Node_ n2);  //2つが同じノードの場合、trueを返す
        bool check_obs(const Node_ node);  //壁の判定
        bool check_parent(const int index, const Node_ node); //親ノードの確認

        double make_heuristic(const Node_ node);  //ヒューリスティック関数の計算
        double sleep_time_;
        double margin_;

        int check_list(const Node_ target_node, std::vector<Node_>& set);  //指定したリストに含まれるか検索
        int search_node_from_list(const Node_ node, std::vector<Node_>& list);  //リストの中を検索

        Node_ set_way_point(const int phase);  //スタートとゴール（経由点）の取得
        Node_ select_min_f();  //oprnリスト内で最もf値が小さいノードを取得
        Node_ get_neighbor_node(const Node_ node, const Motion_ motion);  //隣接ノードの取得
        Motion_ motion(const int dx,const int dy,const int cost);  //モーション
        geometry_msgs::msg::PoseStamped node_to_pose(const Node_ node);  //ノードから座標（ポーズ）に変換

        std::tuple<int, int> search_node(const Node_ node);  //openリストとcloseリストのどっちのリストに入っているのか検索

        double origin_x_;  //原点のx座標
        double origin_y_;  //原点のy座標
        std::vector<double> way_points_x_;  //経由点
        std::vector<double> way_points_y_;  //経由点

        int hz_ = 10;  //周波数
        Node_ start_node_;  //開始位置
        Node_ goal_node_;  //目標位置
        std::vector<Node_> open_list_;  //opneリスト
        std::vector<Node_> close_list_;  //closeリスト

        bool   map_checker_ = false;  //正しくマップはとれたか確認
        bool   test_show_;  //デバッグする場合にはtrue

        //map製作用
        int height_;  //マップの高さ
        int width_;  //マップの幅
        double resolution_;  //マップの解像度
        Node_ origin_;  //マップの原点
        std::vector<std::vector<int>> map_grid_;  //グリッドマップ
        
        //rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock clock_;
        rclcpp::Time begin_;


        //ros::Node_handle
        //std::shared_ptr<rclcpp::Node_> nh_;
        //std::shared_ptr<rclcpp::Node_> private_nh_;
        //rclcpp::Node_::SharedPtr nh;
        //rclcpp::Node_::SharedPtr private_nh;
        //auto node = rclcpp::Node_::make_shared("nh_");
        //auto node = rclcpp::Node_::make_shared("private_nh_");

        // Subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        
        // Publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_node_point_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_current_path_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_new_map_;

        nav_msgs::msg::Path global_path_;
        nav_msgs::msg::OccupancyGrid map_;
        nav_msgs::msg::OccupancyGrid new_map_;
        geometry_msgs::msg::PointStamped current_node_;
};
#endif
