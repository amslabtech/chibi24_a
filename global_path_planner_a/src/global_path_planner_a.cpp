#include "global_path_planner_a/global_path_planner_a.hpp"

using namespace std::chrono_literals;

// コンストラクタ
Astar::Astar() : Node("global_path_planner_a"),  clock_(RCL_ROS_TIME)
{
    // パラメータの取得
    /*
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("test_show", test_show_);
    private_nh_.getParam("sleep_time", sleep_time_);
    private_nh_.getParam("way_points_x", way_points_x_);
    private_nh_.getParam("way_points_y", way_points_y_);
    private_nh_.getParam("margin", margin_);
    */

    declare_parameter<int>("hz", 10); //hz
    get_parameter("hz", hz_);
    declare_parameter<bool>("test_show", true); //デバッグ用
    get_parameter("test_show", test_show_);
    declare_parameter<double>("sleep_time", 0.005);
    get_parameter("sleep_time", sleep_time_);
    declare_parameter<std::vector<double>>("way_points_x", {0, 10, 16.5, 16.5, -17.5, -17.5, 0}); //経由点のx座標
    get_parameter("way_points_x", way_points_x_);
    declare_parameter<std::vector<double>>("way_points_y", {0, 0.5, 0.5, 15.0, 14.5, 0.5, 0}); //経由点のy座標
    get_parameter("way_points_y", way_points_y_);
    declare_parameter<double>("margin", 0.3); //どれくらい拡張するか
    get_parameter("margin", margin_);


    // 基本設定
    // frame idの設定
    global_path_.header.frame_id  = "map";
    current_node_.header.frame_id = "map";

    // dataサイズの確保
    global_path_.poses.reserve(2000);


    // Subscriber
    //sub_map_ = nh_.subscribe("/map", 1, &Astar::map_callback, this);
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable(),
        std::bind(&Astar::map_callback, this, std::placeholders::_1));


    // Publisher
    //pub_path_ = nh_.advertise<nav_msgs::msg::Path>("/global_path", 1);
    //pub_new_map_ = nh_.advertise<nav_msgs::msg::OccupancyGrid>("/map/new_map",1);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/global_path", rclcpp::QoS(1).reliable());
    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map/new_map", rclcpp::QoS(1).reliable());

    if(test_show_)
    {
        //pub_current_path_ = nh_.advertise<nav_msgs::msg::Path>("/current_path", 1);
        //pub_node_point_ = nh_.advertise<geometry_msgs::msg::PointStamped>("/current_node", 1);
        pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/current_path", rclcpp::QoS(1).reliable());
        pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/current_node", rclcpp::QoS(1).reliable());

    }
    
    // timer
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(
    //  static_cast<int>(1.0 / hz_ * 1e3)), std::bind(&Astar::timer_callback, this));
      
}

// timerのコールバック関数（一定間隔でprocessを実行）
void Astar::timer_callback()
{
    process();
}

// mapのコールバック関数
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)  //マップの読み込み
{
    printf("map_callback\n");
    map_ = *msg; //マップを取得
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    height_ = map_.info.height; //マップの高さ[cell]
    width_ = map_.info.width; //マップの幅[cell]
    resolution_ = map_.info.resolution; //マップの解像度[m/cell]
    map_checker_ = true; //マップが取得できたかの確認
}

// hzを返す関数
int Astar::get_freq()
{
    return hz_;
}

// obstacle_expander
void Astar::obs_expander()
{
    RCLCPP_INFO(get_logger(), "obs_expander is runnning...");
    sleep(2);
    new_map_ = map_; //新たなマップを用意
    int grid_size = new_map_.data.size();
    for(int i=0; i<grid_size; i++)
    {
        //障害物がある部分(値が100)の時
        if(map_.data[i] == 100)
        {
            obs_expand(i);
        }
    }
    // new_mapをpublish
    //pub_new_map_.publish(new_map_);
    printf("obs_expand : finish!\n");
    pub_new_map_->publish(new_map_);
}

// 障害物の拡張
void Astar::obs_expand(const int index)
{
    int index_x = index % width_; //x座標を計算
    int index_y = index / width_; //y座標を計算
    int margin_length = round(margin_ / resolution_); //marginのセル数を計算

    // (i, j)からmargin_length範囲内のセルをすべて100に設定
    for(int i=-margin_length; i<margin_length; i++)
    {
        for(int j=-margin_length; j<margin_length; j++)
        {
            int grid_index = (index_x + j) + ((index_y + i) * width_);
            new_map_.data[grid_index] = 100;
        }
    }
}

// スタートとゴールの取得
Node_ Astar::set_way_point(int phase)
{
    Node_ way_point;
    way_point.x = round((way_points_x_[phase] - origin_x_) / resolution_);
    way_point.y = round((way_points_y_[phase] - origin_y_) / resolution_);
    return way_point;
}

// ヒューリスティック関数の計算
double Astar::make_heuristic(const Node_ node)
{
    // ヒューリスティックの重み
    const double w = 1.0;

    // 2点間のユークリッド距離
    const double dx = (node.x - goal_node_.x);
    const double dy = (node.y - goal_node_.y);
    const double distance = hypot(dx,dy);

    return w * distance;
}

// openリスト内で最もf値が小さいノードを取得
Node_ Astar::select_min_f()
{
    Node_ min_node = open_list_[0];
    double min_f = open_list_[0].f;
    //printf("min_f = %lf\n", min_f);

    for(const auto& open_node : open_list_)
    {
        if(open_node.f < min_f)
        {
            min_f = open_node.f;
            min_node = open_node;
        }
    }
    return min_node;
}

// スタートノードの場合、trueを返す
bool Astar::check_start(const Node_ node)
{
    //printf("start\n");
    return check_same_node(node,start_node_);
}

// ゴールノードの場合、trueを返す
bool Astar::check_goal(const Node_ node)
{
    //printf("goal\n");
    return check_same_node(node,goal_node_);
}

// 2つが同じノードである場合、trueを返す
bool Astar::check_same_node(const Node_ n1, const Node_ n2)
{
    if(n1.x == n2.x and n1.y == n2.y)
        return true;
    else
        return false;
}

// 指定したリストに含まれるか検索
int Astar::check_list(const Node_ target_node, std::vector<Node_>& set)
{
    for(int i=0;i<set.size();i++) {
        if(check_same_node(target_node, set[i]))
            return i; // インデックスを返す
    }    
    return -1; //含まれない場合
}

// list1からlist2にノードを移動
void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    const int list1_node_index = check_list(node, list1); // リスト1からノードを探す
    if(list1_node_index == -1)
    {
        RCLCPP_INFO(get_logger(), "swap node was failed");
        exit(0);
    }

    list1.erase(list1.begin() + list1_node_index); // リスト1からノードを削除
    list2.push_back(node); // リスト2にノードを追加
}

// 壁の判定
bool Astar::check_obs(const Node_ node)
{
    const int grid_index = node.x + (node.y * width_);
    return new_map_.data[grid_index] == 100;
}

// 隣接ノードを基にOpenリスト・Closeリストを更新
void Astar::update_list(const Node_ node)
{
    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;

    // 現在のノードを基に隣接ノードを作成
    create_neighbor_nodes(node, neighbor_nodes);

    // Openリスト・Closeリストを更新
    for(const auto& neighbor_node : neighbor_nodes)
    {
        // 障害物の場合
        if(check_obs(neighbor_node))
        {
            continue;
        }

        // リストに同一ノードが含まれるか調べる
        int flag;
        int node_index; //同一ノードのインデックス
        std::tie(flag, node_index) = search_node(neighbor_node); // ノードを探す

        //printf("flag = %d, node_index=%d\n", flag, node_index);

        if(flag == -1) // OpenリストにもCloseリストにもない場合
        {
            open_list_.push_back(neighbor_node);
        }
        else if(flag == 1) // Openリストにある場合
        {
            if(neighbor_node.f < open_list_[node_index].f)
            {
                open_list_[node_index].f = neighbor_node.f;
                open_list_[node_index].parent_x = neighbor_node.parent_x;
                open_list_[node_index].parent_y = neighbor_node.parent_y;
            }

        }
        else if(flag == 2) // Closeリストにある場合
        {
            if(neighbor_node.f < close_list_[node_index].f)
            {
                close_list_.erase(close_list_.begin() + node_index);
                open_list_.push_back(neighbor_node);
            }
        }
    }
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;
    get_motion(motion_list);
    const int motion_num = motion_list.size();

    // 隣接ノードを作成
    for(int i=0; i<motion_num; i++)
    {
        Node_ neighbor_node = get_neighbor_node(node, motion_list[i]); // 隣接ノードを取得
        neighbor_nodes.push_back(neighbor_node);
    }
}

// 動作モデルを作成
void Astar::get_motion(std::vector<Motion_>& list)
{
    list.push_back(motion( 1,  0, 1)); // 前
    list.push_back(motion( 0,  1, 1)); // 左
    list.push_back(motion(-1,  0, 1)); // 後ろ
    list.push_back(motion( 0, -1, 1)); // 右

    list.push_back(motion(  1,  1, sqrt(2))); // 左前
    list.push_back(motion(  1, -1, sqrt(2))); // 右前
    list.push_back(motion( -1,  1, sqrt(2))); // 左後ろ
    list.push_back(motion( -1, -1, sqrt(2))); // 右後ろ
}

// モーション
Motion_ Astar::motion(const int dx,const int dy,const int cost)
{
    // 隣接したグリッドに移動しない場合
    if(1 < abs(dx) or 1 < abs(dy))
    {
        RCLCPP_ERROR_STREAM(get_logger(), "The motion is inappropriate..");
        exit(5);
    }

    Motion_ motion;
    motion.dx = dx;
    motion.dy = dy;
    motion.cost = cost;

    return motion;
}

// 隣接ノードの取得
Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{
    Node_ neighbor_node;

    // 移動
    neighbor_node.x = node.x + motion.dx;
    neighbor_node.y = node.y + motion.dy;

    // f値を記録
    neighbor_node.f = (node.f - make_heuristic(node)) + make_heuristic(neighbor_node) + motion.cost;

    // 親ノードを記録
    neighbor_node.parent_x = node.x;
    neighbor_node.parent_y = node.y;

    return neighbor_node;
}

// OpenリストまたはCloseリストに含まれるか調べる
std::tuple<int, int> Astar::search_node(const Node_ node)
{
    // Openリストに含まれるか検索
    const int open_list_index = search_node_from_list(node, open_list_);
    if(open_list_index != -1)
    {
        return std::make_tuple(1,open_list_index);
    }

    // Closeリストに含まれるか検索
    const int close_list_index = search_node_from_list(node, close_list_);
    if(close_list_index != -1)
    {
        return std::make_tuple(2, close_list_index);
    }
    // OpenリストにもCloseリストにもない場合
    return std::make_tuple(-1, -1);
}

// waypoint間のパスを作成し、グローバルパスに追加
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));
    int count = 0;

    while(not check_start(node))
    {
        for(int i=0; i<close_list_.size(); i++)
        {
            if(check_parent(i,node))
            {
                node = close_list_[i];
                show_node_point(node);
                partial_path.poses.push_back(node_to_pose(node));
                //printf("create_path!\n");
                break;
            }
            if(i == close_list_.size()-1)
            {
                RCLCPP_INFO(get_logger(), "failed.......");
                exit(0);
            }
        }
    }
    reverse(partial_path.poses.begin(), partial_path.poses.end());
    show_path(partial_path);
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
}

// 親ノードの確認
bool Astar::check_parent(const int index, const Node_ node)
{
    bool check_x = close_list_[index].x == node.parent_x;
    bool check_y = close_list_[index].y == node.parent_y;

    return check_x and check_y;
}

// ノードからポーズを計算
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.x * resolution_ + origin_x_;
    pose_stamped.pose.position.y = node.y * resolution_ + origin_y_;
    return pose_stamped;
}

// リストの中を検索
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    for(int i=0; i<list.size(); i++)
        if(check_same_node(node, list[i]))
        {
            return i;
        }
    return -1;
}


// ［デバック用］ノードをRvizに表示
void Astar::show_node_point(const Node_ node)
{
    if(test_show_)
    {
        //printf("sleep_time : %d\n", static_cast<int>(sleep_time_ * 1e9));
        current_node_.point.x = node.x * resolution_ + origin_x_;
        current_node_.point.y = node.y * resolution_ + origin_y_;
        //pub_node_point_.publish(current_node_);
        pub_node_point_->publish(current_node_);
        //ros::Duration(sleep_time_).sleep();
        rclcpp::sleep_for(std::chrono::milliseconds(
            static_cast<int>(sleep_time_ * 1e3)));
    }
}

// ［デバック用］パスをRvizに表示
void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if(test_show_)
    {
        current_path.header.frame_id = "map";
        //pub_current_path_.publish(current_path);
        pub_current_path_->publish(current_path);
        //ros::Duration(sleep_time_).sleep();
        rclcpp::sleep_for(std::chrono::milliseconds(
            static_cast<int>(sleep_time_ * 1e3)));
    }
}

// 実行時間を表示（スタート時間beginを予め設定する）
void Astar::show_exe_time()
{
    RCLCPP_INFO_STREAM(get_logger(), "Duration = " << std::fixed << std::setprecision(2) << clock_.now().seconds() - begin_.seconds() << "s");
}



// 経路計画
void Astar::planning()
{
    begin_ = clock_.now();
    //printf("clock_.now() = %f\n", clock_.now().seconds()); // add
    const int total_phase = way_points_x_.size();
    printf("total_phase = %d\n", total_phase); // add
    for(int phase=0; phase<total_phase-1; phase++)
    {
        printf("phase = %d\n", phase); // add
        open_list_.clear();
        close_list_.clear();

        //printf("open&close_list clear!!\n");

        start_node_ = set_way_point(phase);
        goal_node_ = set_way_point(phase + 1);

        //printf("node_set complete!!\n");

        start_node_.f = make_heuristic(start_node_);
        //printf("start_node_.f : %lf\n", start_node_.f);
        open_list_.push_back(start_node_);
        //printf("push back : complete\n");


        while(rclcpp::ok())
        {
            Node_ min_node = select_min_f();
            show_node_point(min_node);

            if(check_goal(min_node))
            {
                create_path(min_node);
                break;
            }
            else
            {
                swap_node(min_node, open_list_, close_list_);
                update_list(min_node);
            }
        }
    }
    //pub_path_.publish(global_path_);
    pub_path_->publish(global_path_);
    //printf("clock_.now() = %f\n", clock_.now().seconds()); // add
    show_exe_time();
    RCLCPP_INFO(get_logger(), "COMPLITE ASTAR PROGLAM");
    exit(0);
}


// メイン関数で実行する関数
void Astar::process()
{
    RCLCPP_INFO(get_logger(), "process is starting...");

    if(!map_checker_){}
    RCLCPP_INFO(get_logger(), "NOW LOADING...");
    else
    {
        printf("start\n");
        obs_expander(); //壁の判定
        planning(); //グローバルパスの作成
    }

}


