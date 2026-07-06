#ifndef MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
#define MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
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
 *   INIT        → 节点启动，开始软着陆流程
 *   SOFT_LANDING → 施加向下力矩，等待三电机绝对角均 < threshold 持续 stable_duration
 *   READY       → 发布 /delta_arm/ready "READY"，等待 ArmTarget 命令
 *   EXECUTE     → 上抛到目标角度
 *   FAST_RETRACT → 三路上抛电机快速回相对 0（挥拍 windmill 按 launch+strike_delay 独立触发）
 *   TILT_DOWN   → 俯仰抬到击球位；挥拍已发即进 WAIT_WIND_DONE
 *   WAIT_WIND_DONE → 已触发 windmill 开火，等它接住+重力归零完成(重回 WIND_UP)
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
    EXECUTE,
    FAST_RETRACT,
    TILT_DOWN,
    WAIT_WIND_DONE,   // 已触发 windmill 开火，保持俯仰在击球位，等 windmill 接住+重力归零完成(重回 WIND_UP)
    RECOVER_READY
  };

  void controlLoop();

  // ========== 回调 ==========
  void armTargetCallback(const motor_control_ros2::msg::ArmTarget::SharedPtr msg);
  void motorStateCallback(const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);
  void windStatusCallback(const std_msgs::msg::String::SharedPtr msg);

  // ========== 辅助函数 ==========
  void loadConfig(const std::string& config_file);
  void publishCommand(size_t idx,
                      double pos_des, double vel_des, double torque_ff,
                      double kp, double kd, bool bypass_clamp = false);
  void publishTiltCommand(double angle_rel, double vel_des, double torque_ff,
                          double kp, double kd);
  // 简化俯仰：命令角按单一 rate 限速滑向目标 + 恒定向上前馈托住臂自重 + 单一 PD，
  // 抬起/回摆共用一套（恒定前馈抵消臂重后两方向对称）。
  void tiltCommand(double target_rel, double dt);
  bool allMotorsLanded() const;
  bool allMotorsReached() const;
  bool allMotorsAtZero() const;
  bool tiltReached(double target_rad) const;
  // launch 后一过 strike_delay 就发一次 windmill 开火触发，与收拍/抬俯仰进度解耦，
  // 使挥拍时机只由 strike_delay 决定（可早于收拍+抬俯仰总时长）。force=true 时无视时机强发（超时兜底）。
  // 返回是否已发过。
  bool maybeTriggerStrike(bool force = false);
  double estimateLaunchHeight(double delta_rad) const;
  double estimateFallTime(double height_m) const;
  void enterFastRetract();
  void publishReady();

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<motor_control_ros2::msg::ArmTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr state_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr serve_trigger_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wind_status_sub_;  // 订 windmill /serve/status
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ========== 配置参数 ==========
  double downward_torque_;           // 软着陆力矩 (Nm)
  double landing_timeout_;           // 软着陆超时 (s)
  double landing_velocity_threshold_; // 着陆速度阈值 (rad/s)
  double landing_stable_duration_;   // 稳定持续时间 (s)
  double landing_kd_;                // 软着陆阻尼系数
  bool landing_debug_log_;           // 软着陆逐周期数据记录（定位"刚开始的向下力"来源）
  double kp_;
  double kd_;
  double max_velocity_;              // 梯形速度限幅 (rad/s)
  double control_frequency_;         // 控制频率 (Hz)
  double position_tolerance_;        // 到达判定容差 (rad)
  double max_position_error_;        // 位置误差钳位 (rad)
  double tracking_error_pause_;      // 自适应规划暂停阈值 (rad)
  double angle_diff_tolerance_;     // ArmTarget 三路目标角允许差值 (rad)
  std::array<std::string, 3> motor_names_;

  // ========== 运行时状态 ==========
  State state_;
  std::array<double, 3> current_positions_;    // rad
  std::array<double, 3> current_velocities_;   // rad/s
  std::array<bool, 3> motors_online_;
  std::array<bool, 3> has_feedback_;
  std::array<double, 3> last_positions_;       // 用于软着陆变化量判定
  std::array<double, 3> zero_positions_;       // 解耦零点（软着陆完成时锁定）

  std::array<double, 3> target_deltas_rad_;    // 每路 EXECUTE 目标增量（相对零点）
  std::array<double, 3> planned_deltas_rad_;   // 每路梯形规划增量（相对零点）
  std::array<double, 3> current_planned_vels_; // 每路规划器当前速度（限加速度用）
  std::array<double, 3> motor_max_deltas_;     // 每路最大允许增量（实测行程上限）
  double max_acceleration_;                    // 最大加速度 (rad/s²)
  double planner_p_gain_;                      // 减速段比例增益
  // 上抛冲顶部限位：三路满速上抛→接近顶部降速软压→都被挡块顶到同位置、速度一起归零（硬件同步）
  double top_press_margin_rad_;                // 目标压到限位再往上此值，PD 持续把臂顶向挡块
  double top_approach_band_rad_;               // 距限位此范围内判定"已到顶"（撞停对齐判据用）
  double stop_settle_vel_;                     // 撞停判据：三路实际速度都低于此值视为顶到挡块

  rclcpp::Time landing_stable_since_;
  rclcpp::Time init_start_time_;
  rclcpp::Time execute_start_time_;  // EXECUTE 进入时间，用于超时判定
  bool landing_stability_started_;
  bool ready_published_;

  double gravity_compensation_torque_;  // 重力补偿前馈力矩（用户自行调试）
  double top_idle_timeout_;             // EXECUTE 超时时间（s），超时强制进入 DESCENDING

  // ========== 发球测试参数 ==========
  double arm_lower_length_m_;
  double arm_upper_length_m_;
  double gravity_mps2_;
  double strike_plane_drop_m_;
  double strike_timing_offset_s_;
  double strike_delay_override_s_;
  double retract_timeout_s_;
  double tilt_timeout_s_;
  double strike_trigger_pulse_s_;
  double retract_kp_;
  double retract_kd_;
  double retract_torque_ff_;
  double retract_max_velocity_;      // 收拍规划器速度上限 (rad/s)，速度优先，可高于 execute
  double retract_max_acceleration_;  // 收拍规划器加速度上限 (rad/s²)
  double retract_bottom_soft_;       // 底部软着陆阈值 (rad)：实际角低于此值撤下向下前馈
  bool retract_debug_log_;           // 收拍回落逐周期数据记录（定位卡顿用，平时可关）
  double tilt_ready_angle_rad_;      // 待机角（躺平）
  double tilt_down_angle_rad_;       // 击球角（向上抬到位）
  double tilt_kp_;                   // 单一位置增益（抬起/回摆共用）
  double tilt_kd_;                   // 单一速度阻尼
  double tilt_hold_ff_;              // 恒定向上前馈：一直托住臂自重，抬起/回摆全程都挂
  double tilt_rate_rad_s_;           // 命令角滑向目标的单一限速 (rad/s)，防齿轮冲击
  double tilt_position_tolerance_;
  double tilt_max_position_error_;
  std::string tilt_motor_name_;

  // ========== 俯仰电机反馈 ==========
  double tilt_position_;             // 原始反馈角（电机上电坐标系）
  double tilt_velocity_;
  bool tilt_online_;
  bool has_tilt_feedback_;
  double tilt_zero_position_;        // 俯仰零点：首帧反馈时锁定的上电原始角
  bool tilt_zero_captured_;
  double tilt_cmd_angle_;            // 当前下发的俯仰相对角（按 rate 滑向目标）

  rclcpp::Time launch_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time strike_trigger_time_;
  double estimated_launch_height_m_;
  double estimated_fall_time_s_;
  bool strike_trigger_sent_;

  // ========== windmill 联调 ==========
  std::string wind_status_;          // 最近一次收到的 windmill /serve/status 状态名
  bool wind_catch_seen_;             // 本次发球是否已观察到 windmill 进入 CATCH
  double wind_done_timeout_s_;       // 等 windmill 完毕的超时兜底（s），防 windmill 掉线卡死
};

#endif  // MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
