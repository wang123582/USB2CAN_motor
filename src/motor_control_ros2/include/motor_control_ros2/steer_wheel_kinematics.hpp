#ifndef MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_
#define MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_

#include <cmath>
#include <array>

namespace motor_control {

struct WheelCommand {
  double angle;     // 转向角 (度, [0, 360))
  double velocity;  // 轮线速度 (m/s)，最短路径优化后可为负

  WheelCommand() : angle(0.0), velocity(0.0) {}
  WheelCommand(double a, double v) : angle(a), velocity(v) {}
};

/**
 * 四舵轮矩形布局运动学
 *
 * 坐标系（ROS 标准）：X+ = 前，Y+ = 左，wz+ = 逆时针
 *
 * 轮子位置（lx = wheel_base_x/2，ly = wheel_base_y/2）：
 *   FL (左前): (+lx, +ly)
 *   FR (右前): (+lx, -ly)
 *   RL (左后): (-lx, +ly)
 *   RR (右后): (-lx, -ly)
 */
class SteerWheelKinematics {
public:
  SteerWheelKinematics(double wheel_base_x, double wheel_base_y, double wheel_radius);

  void inverseKinematics(
    double vx, double vy, double wz,
    WheelCommand& fl, WheelCommand& fr,
    WheelCommand& rl, WheelCommand& rr) const;

  static double optimizeSteerAngle(
    double current_angle, double target_angle, double& velocity);

  static double normalizeAngle(double angle);
  static double angleDifference(double angle1, double angle2);

  double getWheelBaseX() const { return wheel_base_x_; }
  double getWheelBaseY() const { return wheel_base_y_; }
  double getWheelRadius() const { return wheel_radius_; }

private:
  double wheel_base_x_;
  double wheel_base_y_;
  double wheel_radius_;
  double lx_;
  double ly_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_
