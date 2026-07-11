#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"
#include "motor_control_ros2/steer_wheel_kinematics.hpp"

namespace motor_control {

/**
 * 四舵轮底盘控制节点（矩形布局）
 *
 * 轮子布局（lx = wheel_base_x/2，ly = wheel_base_y/2）：
 *   FL (左前): (+lx, +ly)   FR (右前): (+lx, -ly)
 *   RL (左后): (-lx, +ly)   RR (右后): (-lx, -ly)
 *
 * 订阅：/cmd_vel (Twist)，/dji_motor_states，/chassis/estop
 * 发布：/dji_motor_command_advanced
 *
 * 电机：GM3508（驱动，速度控制）+ GM6020（转向，位置控制）
 * GM3508 减速比通过 drive_gear_ratio 参数配置（默认 19.0）。
 */
class ChassisControlNode : public rclcpp::Node {
public:
  ChassisControlNode() : Node("chassis_control_node")
  {
    std::string config_file;
    try {
      config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2")
        + "/config/omni_chassis_params.yaml";
      loadConfig(config_file);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      throw;
    }

    kinematics_ = std::make_unique<SteerWheelKinematics>(
      wheel_base_x_, wheel_base_y_, wheel_radius_);

    last_cmd_time_ = now();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });

    motor_state_sub_ = create_subscription<motor_control_ros2::msg::DJIMotorState>(
      "/dji_motor_states", 10,
      [this](motor_control_ros2::msg::DJIMotorState::SharedPtr msg) {
        motor_states_[msg->joint_name] = *msg;
      });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/chassis/estop", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        estop_active_ = msg->data;
        if (estop_active_) {
          RCLCPP_WARN(get_logger(), "底盘急停已激活");
        }
      });

    motor_cmd_pub_ = create_publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
      "/dji_motor_command_advanced", 10);

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { controlLoop(); });

    RCLCPP_INFO(get_logger(),
      "chassis_control_node 启动: lx=%.3fm ly=%.3fm wheel_r=%.3fm gear=%.1f freq=%.1fHz",
      wheel_base_x_ / 2.0, wheel_base_y_ / 2.0, wheel_radius_, drive_gear_ratio_, control_frequency_);
    RCLCPP_INFO(get_logger(),
      "FL: %s/%s  FR: %s/%s",
      wheel_configs_[0].steer_name.c_str(), wheel_configs_[0].drive_name.c_str(),
      wheel_configs_[1].steer_name.c_str(), wheel_configs_[1].drive_name.c_str());
    RCLCPP_INFO(get_logger(),
      "RL: %s/%s  RR: %s/%s",
      wheel_configs_[2].steer_name.c_str(), wheel_configs_[2].drive_name.c_str(),
      wheel_configs_[3].steer_name.c_str(), wheel_configs_[3].drive_name.c_str());
  }

private:
  struct WheelConfig {
    std::string steer_name;   // GM6020 转向电机 joint_name
    std::string drive_name;   // GM3508 驱动电机 joint_name
    double steer_offset_deg;  // 编码器零位偏移(度)：正前方时电机编码器示数
    int steer_direction;      // 1 或 -1：正装/反装转向电机
    int drive_direction;      // 1 或 -1：正装/反装驱动电机
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    const double linear_speed = std::hypot(vx, vy);
    if (linear_speed > max_linear_velocity_ && linear_speed > 1e-9) {
      const double scale = max_linear_velocity_ / linear_speed;
      vx *= scale;
      vy *= scale;
    }
    cmd_vx_ = vx;
    cmd_vy_ = vy;
    cmd_wz_ = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    last_cmd_time_ = now();
  }

  void controlLoop()
  {
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_) {
      cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
    }

    const bool has_motion = !estop_active_
      && (std::abs(cmd_vx_) > 0.01 || std::abs(cmd_vy_) > 0.01 || std::abs(cmd_wz_) > 0.01);

    WheelCommand fl_cmd, fr_cmd, rl_cmd, rr_cmd;

    if (has_motion) {
      hold_angles_latched_ = false;
      kinematics_->inverseKinematics(cmd_vx_, cmd_vy_, cmd_wz_,
                                      fl_cmd, fr_cmd, rl_cmd, rr_cmd);

      const double max_ws = std::max(
        {std::abs(fl_cmd.velocity), std::abs(fr_cmd.velocity),
         std::abs(rl_cmd.velocity), std::abs(rr_cmd.velocity)});
      if (max_ws > max_linear_velocity_ && max_ws > 1e-9) {
        const double scale = max_linear_velocity_ / max_ws;
        fl_cmd.velocity *= scale;
        fr_cmd.velocity *= scale;
        rl_cmd.velocity *= scale;
        rr_cmd.velocity *= scale;
      }

      optimizeWheelAngle(0, wheel_configs_[0], fl_cmd);
      optimizeWheelAngle(1, wheel_configs_[1], fr_cmd);
      optimizeWheelAngle(2, wheel_configs_[2], rl_cmd);
      optimizeWheelAngle(3, wheel_configs_[3], rr_cmd);
    } else if (estop_active_) {
      // 急停：锁存一次当前角度并保持原地，不允许自行回零。
      // 注意必须用锁存值：若每周期用实时反馈当目标，手推轮子时目标
      // 跟着反馈走，误差恒为零，转向环完全没有保持力。
      fl_cmd.velocity = fr_cmd.velocity = rl_cmd.velocity = rr_cmd.velocity = 0.0;
      if (!hold_angles_latched_) {
        hold_angles_[0] = currentMechanicalAngle(wheel_configs_[0]);
        hold_angles_[1] = currentMechanicalAngle(wheel_configs_[1]);
        hold_angles_[2] = currentMechanicalAngle(wheel_configs_[2]);
        hold_angles_[3] = currentMechanicalAngle(wheel_configs_[3]);
        hold_angles_latched_ = true;
      }
      fl_cmd.angle = hold_angles_[0];
      fr_cmd.angle = hold_angles_[1];
      rl_cmd.angle = hold_angles_[2];
      rr_cmd.angle = hold_angles_[3];
      for (int i = 0; i < 4; ++i) {
        last_steer_target_[i] = hold_angles_[i];
        last_steer_valid_[i] = true;
      }
    } else {
      // 非急停空闲：转向回机械零位（编码器 = steer_offset，正前方对齐），
      // 这样上电解除急停后轮子自动对正，被推开也会顶回零位。
      hold_angles_latched_ = false;
      fl_cmd.velocity = fr_cmd.velocity = rl_cmd.velocity = rr_cmd.velocity = 0.0;
      fl_cmd.angle = fr_cmd.angle = rl_cmd.angle = rr_cmd.angle = 0.0;
      for (int i = 0; i < 4; ++i) {
        last_steer_target_[i] = 0.0;
        last_steer_valid_[i] = true;
      }
    }

    publishWheelCommand(wheel_configs_[0], fl_cmd);
    publishWheelCommand(wheel_configs_[1], fr_cmd);
    publishWheelCommand(wheel_configs_[2], rl_cmd);
    publishWheelCommand(wheel_configs_[3], rr_cmd);
  }

  double currentMechanicalAngle(const WheelConfig& cfg) const
  {
    const auto it = motor_states_.find(cfg.steer_name);
    if (it == motor_states_.end()) return 0.0;
    return SteerWheelKinematics::normalizeAngle(it->second.angle - cfg.steer_offset_deg);
  }

  void optimizeWheelAngle(int idx, const WheelConfig& cfg, WheelCommand& cmd)
  {
    const double target_mechanical = cmd.angle * static_cast<double>(cfg.steer_direction);
    // 翻转判定参考上一次发出的目标角而非实时反馈：横移目标 ±90° 恰好在
    // 翻转边界（|diff|>90）上，编码器在零位附近的噪声会让判定每次随机选边，
    // 表现为转向轮一次向左一次向右。上次目标是确定量，决策因此可复现。
    double reference;
    if (last_steer_valid_[idx]) {
      reference = last_steer_target_[idx];
    } else {
      const auto it = motor_states_.find(cfg.steer_name);
      if (it == motor_states_.end()) return;
      reference = SteerWheelKinematics::normalizeAngle(
        it->second.angle - cfg.steer_offset_deg);
    }
    cmd.angle = SteerWheelKinematics::optimizeSteerAngle(
      reference, target_mechanical, cmd.velocity);
    last_steer_target_[idx] = cmd.angle;
    last_steer_valid_[idx] = true;
  }

  void publishWheelCommand(const WheelConfig& cfg, const WheelCommand& cmd)
  {
    const rclcpp::Time stamp = now();

    auto steer_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
    steer_msg.header.stamp = stamp;
    steer_msg.joint_name = cfg.steer_name;
    steer_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_POSITION;
    steer_msg.position_target = (cmd.angle + cfg.steer_offset_deg) * M_PI / 180.0;
    motor_cmd_pub_->publish(steer_msg);

    auto drive_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
    drive_msg.header.stamp = stamp;
    drive_msg.joint_name = cfg.drive_name;
    drive_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
    drive_msg.velocity_target =
      (cmd.velocity / wheel_radius_) * drive_gear_ratio_ * static_cast<double>(cfg.drive_direction);
    motor_cmd_pub_->publish(drive_msg);
  }

  void loadConfig(const std::string& path)
  {
    const YAML::Node root = YAML::LoadFile(path);
    YAML::Node p = root;
    for (const char* key : {"chassis_control_node", "omni_chassis_control_node"}) {
      if (root[key] && root[key]["ros__parameters"]) {
        p = root[key]["ros__parameters"];
        break;
      }
    }

    auto loadD = [&p](const std::string& k, double& v) {
      if (p[k]) v = p[k].as<double>();
    };
    auto loadI = [&p](const std::string& k, int& v) {
      if (p[k]) v = p[k].as<int>();
    };
    auto loadS = [&p](const std::string& k, std::string& v) {
      if (p[k]) v = p[k].as<std::string>();
    };

    loadD("control_frequency", control_frequency_);
    loadD("wheel_base_x", wheel_base_x_);
    loadD("wheel_base_y", wheel_base_y_);
    loadD("wheel_radius", wheel_radius_);
    loadD("drive_gear_ratio", drive_gear_ratio_);
    loadD("max_linear_velocity", max_linear_velocity_);
    loadD("max_angular_velocity", max_angular_velocity_);
    loadD("cmd_timeout", cmd_timeout_);
    loadS("cmd_vel_topic", cmd_vel_topic_);

    const std::array<std::string, 4> pfx = {"fl", "fr", "rl", "rr"};
    for (int i = 0; i < 4; ++i) {
      loadS(pfx[i] + "_steer_motor", wheel_configs_[i].steer_name);
      loadS(pfx[i] + "_drive_motor",  wheel_configs_[i].drive_name);
      loadD(pfx[i] + "_steer_offset", wheel_configs_[i].steer_offset_deg);
      loadI(pfx[i] + "_steer_direction", wheel_configs_[i].steer_direction);
      loadI(pfx[i] + "_drive_direction", wheel_configs_[i].drive_direction);
    }
  }

  std::unique_ptr<SteerWheelKinematics> kinematics_;

  std::array<WheelConfig, 4> wheel_configs_ = {{
    {"chassis_fl_steer", "chassis_fl_drive", 0.0, 1, 1},
    {"chassis_fr_steer", "chassis_fr_drive", 0.0, 1, 1},
    {"chassis_rl_steer", "chassis_rl_drive", 0.0, 1, 1},
    {"chassis_rr_steer", "chassis_rr_drive", 0.0, 1, 1},
  }};

  double control_frequency_{100.0};
  double wheel_base_x_{0.40};
  double wheel_base_y_{0.35};
  double wheel_radius_{0.05};
  double drive_gear_ratio_{19.0};
  double max_linear_velocity_{2.0};
  double max_angular_velocity_{3.0};
  double cmd_timeout_{0.5};
  std::string cmd_vel_topic_{"/cmd_vel"};

  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_wz_{0.0};
  std::array<double, 4> hold_angles_{{0.0, 0.0, 0.0, 0.0}};  // 停止时锁存的转向保持角（FL/FR/RL/RR）
  bool hold_angles_latched_{false};
  std::array<double, 4> last_steer_target_{{0.0, 0.0, 0.0, 0.0}};  // 上次发出的转向目标（机械角，翻转判定参考）
  std::array<bool, 4> last_steer_valid_{{false, false, false, false}};
  bool estop_active_{true};
  rclcpp::Time last_cmd_time_;
  std::map<std::string, motor_control_ros2::msg::DJIMotorState> motor_states_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace motor_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::ChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
