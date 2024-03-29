/*
Ray casting 2D grid map
*/

#include "local_map_creator_a/local_map_creator_a.hpp"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creator_a")
{
    /*
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("map_size", map_size_);
    private_nh_.getParam("map_reso", map_reso_);
    */

    //この書き方ちょっと不安
    //declare_parameter("hz", hz_);
    //declare_parameter("map_size", map_size_);
    //declare_parameter("ignore_distance", map_reso_);

    hz_ = this->declare_parameter<int>("hz",10);
    map_size_ = this->declare_parameter<double>("map_size",4.0);
    map_reso_ = this->declare_parameter<double>("ignore_distance",0.025);

    // Subscriber
    //sub_obs_poses_ = nh_.subscribe("/local_map/obstacle", 1, &LocalMapCreator::obs_poses_callback, this);
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/local_map/obstacle", rclcpp::QoS(1).reliable(), std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));

    // Publisher
    //pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacle_pose", rclcpp::QoS(1).reliable());

    // --- 基本設定 ---
    // header
    local_map_.header.frame_id = "base_link";
    // info
    local_map_.info.resolution = map_reso_;
    local_map_.info.width      = int(round(map_size_/map_reso_));
    local_map_.info.height     = int(round(map_size_/map_reso_));
    local_map_.info.origin.position.x = -map_size_/2.0;
    local_map_.info.origin.position.y = -map_size_/2.0;
    // data
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    //printf("<obs_poses_callback>\n");
    obs_poses_      = *msg;
    flag_obs_poses_ = true;
}

/*
// 唯一，main文で実行する関数
void LocalMapCreator::process()
{
    rclcpp::Rate loop_rate(hz_); // 制御周波数の設定

    while(rclcpp::ok())
    {
        if(flag_obs_poses_)
            update_map();  // マップの更新
        //ros::spinOnce();   // コールバック関数の実行
        auto node = rclcpp::Node::make_shared("obstacle_detector");
        rclcpp::spin_some(node);
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}
*/


// マップの更新
void LocalMapCreator::update_map()
{
    init_map(); // マップの初期化
    //printf("〜〜〜start update_map〜〜〜\n");
    for(const auto& obs_pose : obs_poses_.poses)
    {
        const double obs_x     = obs_pose.position.x;
        const double obs_y     = obs_pose.position.y;
        const double obs_dist  = hypot(obs_y, obs_x);
        const double obs_angle = atan2(obs_y, obs_x);

        //printf("hz_ = %d\n",hz_);
        //printf("map_size = %lf\n",map_size_);
        //printf("map_reso = %lf\n",map_reso_);
        printf("obs_x = %lf\n",obs_x);
        printf("obs_y = %lf\n",obs_y);
        printf("obs_dist = %lf\n",obs_dist); //だめ　0.001から変化しない。
        //printf("obs_angle = %lf\n",obs_angle); //ok


        for(double dist_from_start=0.0; (dist_from_start<obs_dist and in_map(dist_from_start, obs_angle)); dist_from_start+=map_reso_)
        {
            //printf("qawsedrftgyhujikolp;\n");
            const int grid_index = get_grid_index(dist_from_start, obs_angle);
            //printf("get_grid_index\n");
            local_map_.data[grid_index] = 0; //「空き」にする
            //printf("grid_index = 0\n");
        }

        if(in_map(obs_dist, obs_angle))
        {
            const int grid_index = xy_to_grid_index(obs_x, obs_y);
            local_map_.data[grid_index] = 100; //「占有」にする
            //printf("grid_index = 100\n");
        }
    }

    pub_local_map_->publish(local_map_);
    //printf("〜〜〜end update_map〜〜〜\n");
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    local_map_.data.clear();

    const int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
    {
        local_map_.data.push_back(-1); //「未知」にする
    }
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    //printf("###start in_map###\n");
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    /*
    printf("dist = %lf\n",dist);
    printf("angle = %lf\n",angle);
    printf("x = %lf\n",x);
    printf("y = %lf\n",y);
    printf("index_x = %d\n",index_x);
    printf("width   = %d\n",local_map_.info.width);
    printf("index_y = %d\n",index_y);
    printf("height  = %d\n",local_map_.info.height);
    */
    

    if(index_x<local_map_.info.width and index_y<local_map_.info.height)
    {
        //printf("###return true###\n");
        return true;
    }
    else
    {
        //printf("###return false###\n");
        return false;
    }
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);

    //printf("<get_grid_index>\n");
    return xy_to_grid_index(x, y);
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    //printf("<xy_to_grid_index>\n");
    return index_x + (index_y * local_map_.info.width);
}

bool LocalMapCreator::get_flag_obs_poses()
{
    //printf("<get_flag_obs_poses>\n");
    return flag_obs_poses_;
}