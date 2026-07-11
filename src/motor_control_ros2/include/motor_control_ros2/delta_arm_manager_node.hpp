#ifndef MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
#define MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "motor_control_ros2/msg/arm_target.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <array>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

/**
 * @brief Delta 机械臂管理器节点
 *
 * 控制 3 路 GO8010 电机，实现以下状态机：
 *   INIT         → 节点启动，开始软着陆流程
 *   SOFT_LANDING → 施加向下力矩，等待三电机绝对角均 < threshold 持续 stable_duration
 *   READY        → 发布 /delta_arm/ready "READY"，等待 ArmTarget 命令
 *   TILT_AIM     → 先确定俯仰：滑向 topic 指定击球角，到位确认后才击球
 *   EXECUTE      → 上抛击球（俯仰保持击球角）
 *   FAST_RETRACT → 三路上抛电机快速回相对 0（俯仰保持击球角）
 *   RECOVER_READY → 俯仰回摆待机，回 READY
 */
class DeltaArmManager : public rclcpp::Node {
public:
  DeltaArmManager();

private:
  // ========== 状态机 ==========
  enum class State {
    INIT,
    SOFT_LANDING,
    READY,
    TILT_AIM,
    EXECUTE,
    FAST_RETRACT,
    RECOVER_READY
  };

  void controlLoop();

  // ========== 回调 ==========
  void armTargetCallback(const motor_control_ros2::msg::ArmTarget::SharedPtr msg);
  void motorStateCallback(const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);

  // ========== 辅助函数 ==========
  void loadConfig(const std::string& config_file);
  void publishCommand(size_t idx,
                      double pos_des, double vel_des, double torque_ff,
                      double kp, double kd, bool bypass_clamp = false);
  void publishTiltCommand(double angle_rel, double vel_des, double torque_ff,
                          double kp, double kd);
  void tiltCommand(double target_rel, double dt);
  bool allMotorsLanded() const;
  bool allMotorsAtZero() const;
  void enterFastRetract();
  void publishReady();

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<motor_control_ros2::msg::ArmTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr state_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ========== 配置参数 ==========
  double downward_torque_;
  double landing_timeout_;
  double landing_velocity_threshold_;
  double landing_stable_duration_;
  double landing_kd_;
  bool landing_debug_log_;
  double kp_;
  double kd_;
  double max_velocity_;
  double control_frequency_;
  double position_tolerance_;
  double max_position_error_;
  double tracking_error_pause_;
  double angle_diff_tolerance_;
  std::array<std::string, 3> motor_names_;

  // ========== 运行时状态 ==========
  State state_;
  std::array<double, 3> current_positions_;
  std::array<double, 3> current_velocities_;
  std::array<bool, 3> motors_online_;
  std::array<bool, 3> has_feedback_;
  std::array<double, 3> zero_positions_;

  std::array<double, 3> target_deltas_rad_;
  std::array<double, 3> planned_deltas_rad_;
  std::array<double, 3> current_planned_vels_;
  std::array<double, 3> motor_max_deltas_;
  double max_acceleration_;
  double planner_p_gain_;
  double top_press_margin_rad_;
  double top_approach_band_rad_;
  double stop_settle_vel_;

  rclcpp::Time landing_stable_since_;
  rclcpp::Time init_start_time_;
  rclcpp::Time execute_start_time_;
  bool landing_stability_started_;
  bool ready_published_;

  double gravity_compensation_torque_;
  double top_idle_timeout_;

  // ========== 运动参数 ==========
  double retract_timeout_s_;
  double tilt_timeout_s_;
  double retract_kp_;
  double retract_kd_;
  double retract_torque_ff_;
  double retract_max_velocity_;
  double retract_max_acceleration_;
  double retract_bottom_soft_;
  bool retract_debug_log_;
  double tilt_ready_angle_rad_;
  double tilt_kp_;
  double tilt_kd_;
  double tilt_hold_ff_;
  double tilt_rate_rad_s_;
  double tilt_position_tolerance_;
  double tilt_max_position_error_;
  std::string tilt_motor_name_;

  // ========== 俯仰电机反馈 ==========
  double tilt_position_;
  double tilt_velocity_;
  bool tilt_online_;
  bool has_tilt_feedback_;
  double tilt_zero_position_;
  bool tilt_zero_captured_;
  double tilt_cmd_angle_;
  double tilt_target_cmd_rad_;  // 本次执行的俯仰目标角，完全由 ArmTarget topic 的 tilt_angle_rad 字段决定

  rclcpp::Time state_enter_time_;
};

#endif  // MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
