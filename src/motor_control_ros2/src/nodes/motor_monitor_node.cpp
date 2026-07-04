#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <map>
#include <vector>
#include <chrono>
#include <cmath>

#include "motor_control_ros2/msg/dji_motor_state.hpp"
#include "motor_control_ros2/msg/damiao_motor_state.hpp"
#include "motor_control_ros2/msg/unitree_motor_state.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include "motor_control_ros2/msg/control_frequency.hpp"

// ANSI 颜色代码
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_DIM     "\033[2m"

// 清屏和光标控制
#define CLEAR_SCREEN  "\033[2J"
#define CURSOR_HOME   "\033[H"
#define CURSOR_HIDE   "\033[?25l"
#define CURSOR_SHOW   "\033[?25h"

class MotorMonitorNode : public rclcpp::Node {
public:
  MotorMonitorNode() : Node("motor_monitor_node") {
    std::cout << CLEAR_SCREEN << CURSOR_HIDE << std::flush;

    heartbeat_timeout_ms_ = 500;

    dji_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorState>(
      "dji_motor_states", 10,
      std::bind(&MotorMonitorNode::djiCallback, this, std::placeholders::_1)
    );

    damiao_sub_ = this->create_subscription<motor_control_ros2::msg::DamiaoMotorState>(
      "damiao_motor_states", 10,
      std::bind(&MotorMonitorNode::damiaoCallback, this, std::placeholders::_1)
    );

    unitree_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeMotorState>(
      "unitree_motor_states", 10,
      std::bind(&MotorMonitorNode::unitreeCallback, this, std::placeholders::_1)
    );

    unitree_go_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&MotorMonitorNode::unitreeGOCallback, this, std::placeholders::_1)
    );

    control_freq_sub_ = this->create_subscription<motor_control_ros2::msg::ControlFrequency>(
      "control_frequency", 10,
      std::bind(&MotorMonitorNode::controlFreqCallback, this, std::placeholders::_1)
    );

    display_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&MotorMonitorNode::updateDisplay, this)
    );

    RCLCPP_INFO(this->get_logger(), "电机监控节点已启动");
  }

  ~MotorMonitorNode() {
    std::cout << CURSOR_SHOW << COLOR_RESET << std::endl;
  }

private:
  struct MotorStats {
    rclcpp::Time last_update;
    double actual_hz = 0.0;
    int msg_count = 0;
    rclcpp::Time last_stat_time;
    bool was_online = false;
  };

  void djiCallback(const motor_control_ros2::msg::DJIMotorState::SharedPtr msg) {
    dji_states_[msg->joint_name] = *msg;
    updateMotorStats(dji_stats_[msg->joint_name]);
    checkOnlineStatus(msg->joint_name, dji_stats_[msg->joint_name], msg->online);
  }

  void damiaoCallback(const motor_control_ros2::msg::DamiaoMotorState::SharedPtr msg) {
    damiao_states_[msg->joint_name] = *msg;
    updateMotorStats(damiao_stats_[msg->joint_name]);
    checkOnlineStatus(msg->joint_name, damiao_stats_[msg->joint_name], msg->online);
  }

  void unitreeCallback(const motor_control_ros2::msg::UnitreeMotorState::SharedPtr msg) {
    unitree_states_[msg->joint_name] = *msg;
    updateMotorStats(unitree_stats_[msg->joint_name]);
    checkOnlineStatus(msg->joint_name, unitree_stats_[msg->joint_name], msg->online);
  }

  void unitreeGOCallback(const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg) {
    unitree_go_states_[msg->joint_name] = *msg;
    updateMotorStats(unitree_go_stats_[msg->joint_name]);
    checkOnlineStatus(msg->joint_name, unitree_go_stats_[msg->joint_name], msg->online);
  }

  void controlFreqCallback(const motor_control_ros2::msg::ControlFrequency::SharedPtr msg) {
    control_freq_ = msg->control_frequency;
    target_freq_ = msg->target_frequency;
  }

  void updateMotorStats(MotorStats& stats) {
    auto now = this->now();

    if (stats.msg_count == 0 && stats.last_update.nanoseconds() == 0) {
      stats.last_update = now;
      stats.last_stat_time = now;
    }

    stats.last_update = now;
    stats.msg_count++;

    auto dt = (now - stats.last_stat_time).seconds();
    if (dt >= 1.0) {
      stats.actual_hz = stats.msg_count / dt;
      stats.msg_count = 0;
      stats.last_stat_time = now;
    }
  }

  void checkOnlineStatus(const std::string& name, MotorStats& stats, bool current_online) {
    if (stats.last_update.nanoseconds() == 0) {
      stats.was_online = false;
      return;
    }

    bool is_online = current_online;

    if (is_online && !stats.was_online) {
      RCLCPP_INFO(this->get_logger(), "%s[%s%s 上线%s]%s",
                  COLOR_GREEN, COLOR_BOLD, name.c_str(), COLOR_RESET, COLOR_RESET);
    } else if (!is_online && stats.was_online) {
      RCLCPP_WARN(this->get_logger(), "%s[%s%s 离线%s]%s",
                  COLOR_RED, COLOR_BOLD, name.c_str(), COLOR_RESET, COLOR_RESET);
    }

    stats.was_online = is_online;
  }

  bool isMotorOnline(const MotorStats& stats, bool msg_online) {
    if (stats.last_update.nanoseconds() == 0) {
      return false;
    }

    auto now = this->now();
    auto dt_ms = (now - stats.last_update).seconds() * 1000.0;

    if (dt_ms >= heartbeat_timeout_ms_) {
      return false;
    }

    return msg_online;
  }

  void updateDisplay() {
    std::ostringstream oss;

    oss << CURSOR_HOME;

    oss << COLOR_BOLD << COLOR_CYAN
        << "╔═══════════════════════════════════════════════════════════════════════════════╗\n"
        << "║                        电机控制系统实时监控                                   ║\n"
        << "╚═══════════════════════════════════════════════════════════════════════════════╝"
        << COLOR_RESET << "\n\n";

    oss << COLOR_DIM << "控制频率: ";
    if (control_freq_ > 0) {
      if (control_freq_ >= target_freq_ * 0.95) {
        oss << COLOR_GREEN;
      } else if (control_freq_ >= target_freq_ * 0.8) {
        oss << COLOR_YELLOW;
      } else {
        oss << COLOR_RED;
      }
      oss << std::setprecision(1) << control_freq_ << " Hz" << COLOR_DIM
          << "  目标: " << target_freq_ << " Hz";
    } else {
      oss << COLOR_RED << "等待数据...";
    }
    oss << COLOR_RESET << "\n\n";

    if (!dji_states_.empty()) {
      oss << COLOR_BOLD << COLOR_YELLOW << "【DJI 电机】" << COLOR_RESET << "\n";
      oss << COLOR_DIM
          << "┌─────────────┬────────┬────────┬─────────┬──────┬────────┐\n"
          << "│ 名称        │ 型号   │ 状态   │ 角度(°) │ 温度 │ 频率   │\n"
          << "├─────────────┼────────┼────────┼─────────┼──────┼────────┤"
          << COLOR_RESET << "\n";

      for (const auto& [name, state] : dji_states_) {
        bool is_online = isMotorOnline(dji_stats_[name], state.online);
        std::string status_color = is_online ? COLOR_GREEN : COLOR_RED;
        std::string status_text = is_online ? "在线" : "离线";

        std::string freq_color = COLOR_GREEN;
        if (state.control_frequency > 0) {
          if (state.control_frequency < 400) freq_color = COLOR_RED;
          else if (state.control_frequency < 475) freq_color = COLOR_YELLOW;
        }

        oss << "│ " << std::left << std::setw(11) << name << " │ "
            << std::setw(6) << state.model << " │ "
            << status_color << std::setw(6) << status_text << COLOR_RESET << " │ "
            << std::right << std::setw(7) << std::fixed << std::setprecision(1) << state.angle << " │ "
            << std::setw(4) << (int)state.temperature << "°C │ "
            << freq_color << std::setw(5) << std::setprecision(0) << state.control_frequency << "Hz" << COLOR_RESET << " │\n";
      }

      oss << COLOR_DIM
          << "└─────────────┴────────┴────────┴─────────┴──────┴────────┘"
          << COLOR_RESET << "\n\n";
    }

    if (!damiao_states_.empty()) {
      oss << COLOR_BOLD << COLOR_MAGENTA << "【达妙电机】" << COLOR_RESET << "\n";
      oss << COLOR_DIM
          << "┌─────────────┬────────┬──────────┬──────────┬──────────┬──────┬────────┐\n"
          << "│ 名称        │ 状态   │ 角度(°)  │ 速度(r/s)│ 力矩(Nm) │ 温度 │ 频率   │\n"
          << "├─────────────┼────────┼──────────┼──────────┼──────────┼──────┼────────┤"
          << COLOR_RESET << "\n";

      for (const auto& [name, state] : damiao_states_) {
        bool is_online = isMotorOnline(damiao_stats_[name], state.online);
        std::string status_color = is_online ? COLOR_GREEN : COLOR_RED;
        std::string status_text = is_online ? "在线" : "离线";

        oss << "│ " << std::left << std::setw(11) << name << " │ "
            << status_color << std::setw(6) << status_text << COLOR_RESET << " │ "
          << std::right << std::setw(8) << std::fixed << std::setprecision(3) << (state.position * 180.0 / M_PI) << " │ "
            << std::setw(8) << std::setprecision(2) << state.velocity << " │ "
            << std::setw(8) << std::setprecision(2) << state.torque << " │ "
            << std::setw(4) << (int)state.temp_mos << "°C │ "
            << std::setw(5) << std::setprecision(0) << damiao_stats_[name].actual_hz << "Hz │\n";
      }

      oss << COLOR_DIM
          << "└─────────────┴────────┴──────────┴──────────┴──────────┴──────┴────────┘"
          << COLOR_RESET << "\n\n";
    }

    if (!unitree_states_.empty() || !unitree_go_states_.empty()) {
      oss << COLOR_BOLD << COLOR_BLUE << "【宇树电机】" << COLOR_RESET << "\n";
      oss << COLOR_DIM
          << "┌─────────────┬──────────┬────┬────────┬──────────┬──────────┬──────────┬──────┬──────┬────────┐\n"
          << "│ 名称        │ 型号     │ ID │ 状态   │ 角度(°)  │ 速度(r/s)│ 力矩(Nm) │ 温度 │ 错误 │ 频率   │\n"
          << "├─────────────┼──────────┼────┼────────┼──────────┼──────────┼──────────┼──────┼──────┼────────┤"
          << COLOR_RESET << "\n";

      for (const auto& [name, state] : unitree_states_) {
        bool is_online = isMotorOnline(unitree_stats_[name], state.online);
        std::string status_color = is_online ? COLOR_GREEN : COLOR_RED;
        std::string status_text = is_online ? "在线" : "离线";

        oss << "│ " << std::left << std::setw(11) << name << " │ "
            << std::setw(8) << "A1" << " │ "
            << std::right << std::setw(2) << (int)state.motor_id << " │ "
            << status_color << std::setw(6) << status_text << COLOR_RESET << " │ "
          << std::right << std::setw(8) << std::fixed << std::setprecision(3) << (state.position * 180.0 / M_PI) << " │ "
            << std::setw(8) << std::setprecision(2) << state.velocity << " │ "
            << std::setw(8) << std::setprecision(2) << state.torque << " │ "
            << std::setw(4) << (int)state.temperature << "°C │ "
            << (state.error ? COLOR_RED : COLOR_DIM) << std::setw(4) << (int)state.error << COLOR_RESET << " │ "
            << std::setw(5) << std::setprecision(0) << unitree_stats_[name].actual_hz << "Hz │\n";
      }

      for (const auto& [name, state] : unitree_go_states_) {
        bool is_online = isMotorOnline(unitree_go_stats_[name], state.online);
        std::string status_color = is_online ? COLOR_GREEN : COLOR_RED;
        std::string status_text = is_online ? "在线" : "离线";

        oss << "│ " << std::left << std::setw(11) << name << " │ "
            << std::setw(8) << "GO-8010" << " │ "
            << std::right << std::setw(2) << (int)state.motor_id << " │ "
            << status_color << std::setw(6) << status_text << COLOR_RESET << " │ "
          << std::right << std::setw(8) << std::fixed << std::setprecision(3) << (state.position * 180.0 / M_PI) << " │ "
            << std::setw(8) << std::setprecision(2) << state.velocity << " │ "
            << std::setw(8) << std::setprecision(2) << state.torque << " │ "
            << std::setw(4) << (int)state.temperature << "°C │ "
            << (state.error ? COLOR_RED : COLOR_DIM) << std::setw(4) << (int)state.error << COLOR_RESET << " │ "
            << std::setw(5) << std::setprecision(0) << unitree_go_stats_[name].actual_hz << "Hz │\n";
      }

      oss << COLOR_DIM
          << "└─────────────┴──────────┴────┴────────┴──────────┴──────────┴──────────┴──────┴──────┴────────┘"
          << COLOR_RESET << "\n\n";
    }

    oss << COLOR_DIM << "提示: 按 Ctrl+C 退出监控  |  心跳超时: " << heartbeat_timeout_ms_ << "ms" << COLOR_RESET << "\n";

    std::cout << oss.str() << std::flush;
  }

private:
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr dji_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DamiaoMotorState>::SharedPtr damiao_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeMotorState>::SharedPtr unitree_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr unitree_go_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::ControlFrequency>::SharedPtr control_freq_sub_;

  rclcpp::TimerBase::SharedPtr display_timer_;

  std::map<std::string, motor_control_ros2::msg::DJIMotorState> dji_states_;
  std::map<std::string, motor_control_ros2::msg::DamiaoMotorState> damiao_states_;
  std::map<std::string, motor_control_ros2::msg::UnitreeMotorState> unitree_states_;
  std::map<std::string, motor_control_ros2::msg::UnitreeGO8010State> unitree_go_states_;

  std::map<std::string, MotorStats> dji_stats_;
  std::map<std::string, MotorStats> damiao_stats_;
  std::map<std::string, MotorStats> unitree_stats_;
  std::map<std::string, MotorStats> unitree_go_stats_;

  double control_freq_ = 0.0;
  double target_freq_ = 0.0;

  double heartbeat_timeout_ms_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
