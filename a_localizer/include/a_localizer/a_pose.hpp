/**
 * @file pose.h
 * @author Takuma Tanaka
 * @brief 
 * @version 0.1
 * @date 2023-09-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef A_POSE_HPP
#define A_POSE_HPP

#include <cmath>

/**
 * @brief class for pose
 */
class Pose //: public rclcpp::Node
{
public:
    Pose(); // デフォルトコンストラクタ
    Pose(const double x, const double y, const double yaw); // コンストラクタ
    Pose& operator =(const Pose& pose); // 代入演算子
    Pose& operator /=(const double a);  // 複合代入演算子/=

    // accessor
    void set(const double x, const double y, const double yaw);
    double x()   const { return x_; }
    double y()   const { return y_; }
    double yaw() const { return yaw_; }

    // ノイズを含む移動
    void move(double length, double direction, double rotation, const double fw_noise, const double rot_noise);

    // 適切な角度(-M_PI ~ M_PI)に変更
    void normalize_angle();

private:

    double x_;   // [m]
    double y_;   // [m]
    double yaw_; // [rad]
};

#endif  // A_POSE_HPP