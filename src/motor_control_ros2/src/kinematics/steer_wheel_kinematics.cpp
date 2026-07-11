#include "motor_control_ros2/steer_wheel_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace motor_control {

SteerWheelKinematics::SteerWheelKinematics(
  double wheel_base_x, double wheel_base_y, double wheel_radius)
: wheel_base_x_(wheel_base_x),
  wheel_base_y_(wheel_base_y),
  wheel_radius_(wheel_radius),
  lx_(wheel_base_x / 2.0),
  ly_(wheel_base_y / 2.0)
{
}

void SteerWheelKinematics::inverseKinematics(
  double vx, double vy, double wz,
  WheelCommand& fl, WheelCommand& fr,
  WheelCommand& rl, WheelCommand& rr) const
{
  const double vx_fl = vx - wz * ly_;  // FL: (+lx, +ly)
  const double vy_fl = vy + wz * lx_;
  fl.velocity = std::sqrt(vx_fl * vx_fl + vy_fl * vy_fl);
  fl.angle = normalizeAngle(std::atan2(vy_fl, vx_fl) * 180.0 / M_PI);

  const double vx_fr = vx + wz * ly_;  // FR: (+lx, -ly)
  const double vy_fr = vy + wz * lx_;
  fr.velocity = std::sqrt(vx_fr * vx_fr + vy_fr * vy_fr);
  fr.angle = normalizeAngle(std::atan2(vy_fr, vx_fr) * 180.0 / M_PI);

  const double vx_rl = vx - wz * ly_;  // RL: (-lx, +ly)
  const double vy_rl = vy - wz * lx_;
  rl.velocity = std::sqrt(vx_rl * vx_rl + vy_rl * vy_rl);
  rl.angle = normalizeAngle(std::atan2(vy_rl, vx_rl) * 180.0 / M_PI);

  const double vx_rr = vx + wz * ly_;  // RR: (-lx, -ly)
  const double vy_rr = vy - wz * lx_;
  rr.velocity = std::sqrt(vx_rr * vx_rr + vy_rr * vy_rr);
  rr.angle = normalizeAngle(std::atan2(vy_rr, vx_rr) * 180.0 / M_PI);
}

double SteerWheelKinematics::optimizeSteerAngle(
  double current_angle, double target_angle, double& velocity)
{
  current_angle = normalizeAngle(current_angle);
  target_angle = normalizeAngle(target_angle);
  const double diff = angleDifference(target_angle, current_angle);
  if (std::abs(diff) > 90.0) {
    velocity = -velocity;
    target_angle = normalizeAngle(target_angle + 180.0);
  }
  return target_angle;
}

double SteerWheelKinematics::normalizeAngle(double angle)
{
  angle = std::fmod(angle, 360.0);
  if (angle < 0.0) {
    angle += 360.0;
  }
  return angle;
}

double SteerWheelKinematics::angleDifference(double angle1, double angle2)
{
  double diff = angle1 - angle2;
  while (diff > 180.0) diff -= 360.0;
  while (diff <= -180.0) diff += 360.0;
  return diff;
}

}  // namespace motor_control
