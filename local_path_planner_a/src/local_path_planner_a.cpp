/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner_a/local_path_planner_a.hpp"

using namespace std::chrono_literals;

// コンストラクタ
DWAPlanner::DWAPlanner() : Node("local_path_planner_a"), clock_(RCL_ROS_TIME)
{
    // パラメータの取得 
    // パスを可視化するかの設定用
    declare_parameter<bool>("is_visible", true);
    get_parameter("is_visible", is_visible_);

    // 制御周波数 [Hz]
    declare_parameter<int>("hz", 10);
    get_parameter("hz", hz_);
    
    // frame_id
    declare_parameter<std::string>("robot_frame", "base_link"); 
    get_parameter("robot_frame", robot_frame_);

    // 速度制限
    declare_parameter<double>("max_vel1", 0.35); // [m/s]（平常時）//0.35
    get_parameter("max_vel1", max_vel1_);
    declare_parameter<double>("max_vel2", 0.25); //[m/s]（減速時）//0.25
    get_parameter("max_vel2", max_vel2_);
    declare_parameter<double>("avoid_thres_vel", 0.25); //[m/s]（回避中か判断する閾値）
    get_parameter("avoid_thres_vel", avoid_thres_vel_);
    declare_parameter<double>("min_vel", 0.15); //[m/s] //0.0あげてみた
    get_parameter("min_vel", min_vel_);
    declare_parameter<double>("max_yawrate1", 0.8); //[rad/s]（平常時）//1.0
    get_parameter("max_yawrate1", max_yawrate1_);
    declare_parameter<double>("max_yawrate2", 0.45); //[rad/s]（減速時）//0.55 0.75
    get_parameter("max_yawrate2", max_yawrate2_);
    declare_parameter<double>("turn_thres_yawrate", 0.25); //[rad/s]（旋回中か判断する閾値）
    get_parameter("turn_thres_yawrate", turn_thres_yawrate_);
    declare_parameter<double>("mode_log_time", 6.0); //5.0
    get_parameter("mode_log_time", mode_log_time_);

    // 加速度制限
    declare_parameter<double>("max_accel", 1000.0); //[m/s^2]
    get_parameter("max_accel", max_accel_);
    declare_parameter<double>("max_dyawrate", 1000.0); //[rad/s^2]
    get_parameter("max_dyawrate", max_dyawrate_);

    // 速度解像度
    declare_parameter<double>("vel_reso", 0.05); //[m/s]
    get_parameter("vel_reso", vel_reso_);
    declare_parameter<double>("yawrate_reso", 0.02); //[rad/s]
    get_parameter("yawrate_reso", yawrate_reso_);

    // 停止状態か判断する閾値
    declare_parameter<double>("stop_vel_th", 0.1);    
    get_parameter("stop_vel_th", stop_vel_th_);
    declare_parameter<double>("stop_yawrate_th", 0.1);    
    get_parameter("stop_yawrate_th", stop_yawrate_th_);

    // 時間 [s]
    declare_parameter<double>("dt", 0.1);    
    get_parameter("dt", dt_);
    declare_parameter<double>("predict_time1", 4.0); //2.0    
    get_parameter("predict_time1", predict_time1_);
    declare_parameter<double>("predict_time2", 6.0); //6.0    
    get_parameter("predict_time2", predict_time2_);

    // 機体サイズ（半径）[m]
    declare_parameter<double>("roomba_radius", 0.20); // 0.25
    get_parameter("roomba_radius", roomba_radius_);
    declare_parameter<double>("radius_margin1", 0.1); //（平常時）//0.1
    get_parameter("radius_margin1", radius_margin1_);
    declare_parameter<double>("radius_margin2", 0.04); //（減速時）//0.04
    get_parameter("radius_margin2", radius_margin2_);

    // 重み定数 [-]
    declare_parameter<double>("weight_heading1", 0.8); //（平常時）  //0.85  
    get_parameter("weight_heading1", weight_heading1_);
    declare_parameter<double>("weight_heading2", 0.65);  //（減速時） //0.7  
    get_parameter("weight_heading2", weight_heading2_);
    declare_parameter<double>("weight_dist1", 1.1); //（平常時) 1.5
    get_parameter("weight_dist1", weight_dist1_);
    declare_parameter<double>("weight_dist2", 0.7); //（減速時） 0.8
    get_parameter("weight_dist2", weight_dist2_);
    declare_parameter<double>("weight_vel", 0.6);    
    get_parameter("weight_vel", weight_vel_);  
    
    // 許容誤差 [m]
    declare_parameter<double>("goal_tolerance", 1.0);    
    get_parameter("goal_tolerance", goal_tolerance_);
    
    // 評価関数distで探索する範囲[m]
    declare_parameter<double>("search_range", 0.95);    
    get_parameter("search_range", search_range_);


    // tf_buffer_とtf_listenerを初期化する
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber
    //sub_local_goal_ = nh_.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    //sub_obs_poses_  = nh_.subscribe("/local_map/obstacle", 1, &DWAPlanner::obs_poses_callback, this);
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/local_goal", rclcpp::QoS(1).reliable(),
        std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/local_map/obstacle", rclcpp::QoS(1).reliable(), 
        std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

/*
obstacle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/local_map/obstacle", rclcpp::QoS(1).reliable());
pub_local_goal_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/local_goal", rclcpp::QoS(1).reliable());
*/

    // Publisher
    //pub_cmd_speed_    = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    //pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
    //pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
    pub_cmd_speed_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
        "/roomba/control", rclcpp::QoS(1).reliable());
    pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/predict_local_paths", rclcpp::QoS(1).reliable());
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/optimal_local_path", rclcpp::QoS(1).reliable());

    // timer
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(
    //  static_cast<int>(1.0 / hz_ * 1e3)), std::bind(&DWAPlanner::timer_callback, this));
}

/*
エラー内容
unable to obtain a transform : Lookup would require extrapolation into the past. 
Requested time 1709795605.433078 but the earliest data is at time 1712475294.363233, when looking up transform from frame [map] to frame [base_link]

(map) bagファイルを元に作成したlocal_goal
/clockの中身を確認。この時間はuse_sim_time:=Trueにより、bagファイルに記録された時間になっている。ex)1709795605.433078

(base_link)
rviz2に表示される時間ROS_Timeは現在の時間になっている。ex)1712475294.363233

これらの時刻の整合性が取れていないため、上記のエラーが表示されているのだと考えられる。実環境だと動くかも？

*/



// local_goalのコールバック関数
// local_goalはマップ座標系(map)だが、実際の移動に合わせるためにルンバ座標系(base_link)に変換する処理を行う
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(robot_frame_, "map", tf2::TimePointZero); //rclcpp::Time(0)でも可能かも？この書き方は問題ない。
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "unable to obtain a transform : %s", ex.what());
        flag_local_goal_ = false;
        return;
    }
    tf2::doTransform(*msg, local_goal_, transform);
}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    //printf("obs_poses_callback\n");
    obs_poses_      = *msg;
    flag_obs_poses_ = true;
}

// hzを返す関数
int DWAPlanner::get_freq()
{
    return hz_;
}

/*
void DWAPlanner::timer_callback()
{
    process();
}
*/

// 唯一，main文で実行する関数
void DWAPlanner::process()
{
    //auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); //こいつが原因 class内で定義した
        //tf2_ros::TransformListener tf_listener(tf_buffer_);

    if(can_move())
    {
        const std::vector<double> input = calc_final_input();
        roomba_control(input[0], input[1]);
    }
    else
    {
        roomba_control(0.0, 0.0);
    }
}

// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    if(!(flag_local_goal_ && flag_obs_poses_)) 
    {
        return false; // msg受信済みか
    }
    
    const double dx = local_goal_.point.x;
    const double dy = local_goal_.point.y;
    const double dist_to_goal = hypot(dx, dy); // 現在位置からゴールまでの距離

    if(dist_to_goal > goal_tolerance_)
    {
        return true;
    }
    else
    {
        roomba_control(0.0, 0.0);
        exit(0); // ノード終了
    }
}

// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    cmd_speed_.mode           = 11; // 任意の動作を実行するモード
    cmd_speed_.cntl.linear.x  = velocity;
    cmd_speed_.cntl.angular.z = yawrate;

    pub_cmd_speed_->publish(cmd_speed_);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    // 並進速度と旋回速度のすべての組み合わせを評価
    int i = 0; // 現在の軌跡のインデックス保持用
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {
            const std::vector<State> trajectory = calc_traj(velocity, yawrate); // 予測軌跡の生成
            double score = calc_evaluation(trajectory); // 予測軌跡に対する評価値の計算
            trajectories.push_back(trajectory);

            if(velocity<stop_vel_th_ and abs(yawrate)<stop_yawrate_th_)
                score = -1e6;

            // 最大値の更新
            if(max_score < score)
            {
                max_score = score;
                input[0]  = velocity;
                input[1]  = yawrate;
                index_of_max_score = i;
            }
            i++;
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // pathの可視化
    if(is_visible_)
    {
        rclcpp::Time now =  get_clock()->now(); //clock_.now();
        for(i=0; i<trajectories.size(); i++)
        {
            if(i == index_of_max_score)
                visualize_traj(trajectories[i], pub_optimal_path_, now);
            else
                visualize_traj(trajectories[i], pub_predict_path_, now);
        }
    }

    return input;
}

// 旋回状況に応じた減速機能
void DWAPlanner::change_mode()
{
    if(abs(roomba_.yawrate)>turn_thres_yawrate_ or roomba_.velocity<avoid_thres_vel_)
        mode_log_.push_back(2.0); // 減速モード
    else
        mode_log_.push_back(2.0); // 常に減速モード
        //mode_log_.push_back(1.0);

    if(mode_log_.size() > hz_*mode_log_time_)
        mode_log_.erase(mode_log_.begin());

    double mode_sum = 0.0;
    for(const auto& mode : mode_log_)
    {
        mode_sum += mode;
    }

    double mode_avg = mode_sum/mode_log_.size();

    if(mode_avg < 1.5) // 平常時
    {
        mode_           = 1;
        max_vel_        = max_vel1_;
        max_yawrate_    = max_yawrate1_;
        radius_margin_  = radius_margin1_;
        weight_heading_ = weight_heading1_;
        weight_dist_    = weight_dist1_;
        predict_time_   = predict_time1_;
        // std::cout << "減速OFF" << std::endl;
    }
    else // 減速時
    {
        mode_           = 2;
        max_vel_        = max_vel2_;
        max_yawrate_    = max_yawrate2_;
        radius_margin_  = radius_margin2_;
        weight_heading_ = weight_heading2_;
        weight_dist_    = weight_dist2_;
        predict_time_   = predict_time2_;
        // std::cout << "減速ON" << std::endl;
    }
}

// Dynamic Windowを計算
void DWAPlanner::calc_dynamic_window()
{
    // 車両モデルによるWindow
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // 運動モデルによるWindow
    double Vd[] = {roomba_.velocity - max_accel_*dt_,
                   roomba_.velocity + max_accel_*dt_,
                   roomba_.yawrate  - max_dyawrate_*dt_,
                   roomba_.yawrate  + max_dyawrate_*dt_};

    // 最終的なDynamic Window
    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 予測軌跡を作成
std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> trajectory;           // 軌跡格納用の動的配列
    State state = {0.0, 0.0, 0.0, 0.0, 0.0}; // 軌跡作成用の仮想ロボット

    // 軌跡を格納
    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.yaw      += yawrate * dt_;
    state.yaw       = normalize_angle(state.yaw);
    state.x        += velocity * cos(state.yaw) * dt_;
    state.y        += velocity * sin(state.yaw) * dt_;
    state.velocity  = velocity;
    state.yawrate   = yawrate;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double DWAPlanner::normalize_angle(double angle)
{
    while(M_PI  < angle) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// 評価関数を計算
double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    // 最終時刻のロボットの方位
    const double theta = traj.back().yaw;

    // 最終時刻の位置に対するゴールの方位
    const double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

    // ゴールまでの方位差分
    double target_theta = 0.0;
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;

    // headingの評価値
    const double heading_eval = (M_PI - abs(normalize_angle(target_theta)))/M_PI; // 正規化

    return heading_eval;
}

// distの評価関数を計算
double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    double min_dist = search_range_; // 最も近い障害物との距離

    // pathの点と障害物のすべての組み合わせを探索
    for(const auto& state : traj)
    {
        for(const auto& obs_pose : obs_poses_.poses)
        {
            // pathのうちの１点と障害物の距離を計算
            const double dx   = obs_pose.position.x - state.x;
            const double dy   = obs_pose.position.y - state.y;
            const double dist = hypot(dx, dy);

            // 壁に衝突したパスを評価
            if(dist <= roomba_radius_+radius_margin_)
                return -1e6;

            // 最小値の更新
            if(dist < min_dist)
                min_dist = dist;
        }
    }

    return min_dist/search_range_; // 正規化
}

// velocityの評価関数を計算
double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    if(0.0 < traj.back().velocity) // 前進
        return traj.back().velocity/max_vel_; // 正規化
    else // 後退
        return 0.0;
}

// 軌跡を可視化
void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now)
{
    nav_msgs::msg::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = robot_frame_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = robot_frame_;

    for(const auto& state : traj)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }

    pub_local_path->publish(local_path);
}