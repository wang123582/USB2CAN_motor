#include "motor_control_ros2/remote_control_node.hpp"
#include <algorithm>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace motor_control {

RemoteControlNode::RemoteControlNode() : Node("remote_control_node") {
    RCLCPP_INFO(this->get_logger(), "正在初始化遥控器节点...");

    std::string config_file;
    try {
        config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") +
            "/config/remote_control_params.yaml";
        RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());
        loadRemoteControlParams(config_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "遥控器节点初始化失败: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "配置文件路径: %s", config_file.c_str());
        throw;
    }

    current_speed_scale_ = config_.default_speed_scale;
    last_arm_press_time_ = this->now();

    // 创建订阅者
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&RemoteControlNode::joyCallback, this, std::placeholders::_1)
    );
    arm_ready_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/delta_arm/ready", 10,
        std::bind(&RemoteControlNode::readyCallback, this, std::placeholders::_1)
    );

    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );
    arm_target_pub_ = this->create_publisher<motor_control_ros2::msg::ArmTarget>(
        "/delta_arm/target", 10
    );
    
    // 创建定时发布器
    auto period = std::chrono::duration<double>(1.0 / config_.publish_frequency);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RemoteControlNode::publishTimer, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "遥控器节点启动成功");
    RCLCPP_INFO(this->get_logger(), 
        "轴配置 - 前后: %d, 左右: %d, 旋转: %d",
        config_.axis_forward, config_.axis_strafe, config_.axis_rotate);
    RCLCPP_INFO(this->get_logger(), 
        "按钮配置 - 加速: %d, 减速: %d, 停止: %d",
        config_.button_speed_up, config_.button_speed_down, config_.button_stop);
    RCLCPP_INFO(this->get_logger(), 
        "速度配置 - 线速度上限: %.2f m/s, 角速度上限: %.2f rad/s",
        config_.max_linear_velocity, config_.max_angular_velocity);
    RCLCPP_INFO(this->get_logger(),
        "默认速度缩放因子: %.1f%%, 死区: %.2f",
        config_.default_speed_scale * 100.0, config_.deadzone);
    RCLCPP_INFO(this->get_logger(),
        "delta_arm 扳机控制: axis=%d range=[%.3f, %.3f] rad trigger=%.2f reset=%.2f",
        config_.arm_trigger_axis,
        config_.arm_min_angle_rad,
        config_.arm_max_angle_rad,
        config_.arm_trigger_threshold,
        config_.arm_trigger_reset_threshold);
}

void RemoteControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // 检查按钮索引是否有效
    if (msg->buttons.size() <= static_cast<size_t>(std::max({
        config_.button_speed_up, 
        config_.button_speed_down, 
        config_.button_stop}))) {
        return;
    }
    
    // 检查轴索引是否有效
    if (msg->axes.size() <= static_cast<size_t>(std::max({
        config_.axis_forward,
        config_.axis_strafe,
        config_.axis_rotate,
        config_.arm_trigger_axis}))) {
        return;
    }
    
    // 速度缩放调整（检测按钮按下事件，而不是按住）
    // 加速按钮：每按一下增加 10%
    if (msg->buttons[config_.button_speed_up] && !last_button_speed_up_) {
        current_speed_scale_ = std::min(20.0,  // 最大 1000%
            current_speed_scale_ + config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "加速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_up_ = msg->buttons[config_.button_speed_up];
    
    // 减速按钮：每按一下减少 10%
    if (msg->buttons[config_.button_speed_down] && !last_button_speed_down_) {
        current_speed_scale_ = std::max(0.0,  // 最小 0%
            current_speed_scale_ - config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "减速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_down_ = msg->buttons[config_.button_speed_down];
    
    // 紧急停止按钮（检测按下事件）
    if (msg->buttons[config_.button_stop] && !last_button_stop_) {
        last_cmd_.linear.x = 0.0;
        last_cmd_.linear.y = 0.0;
        last_cmd_.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "紧急停止");
    }
    last_button_stop_ = msg->buttons[config_.button_stop];
    
    // 读取摇杆输入并应用死区
    double forward = applyDeadzone(msg->axes[config_.axis_forward], config_.deadzone);
    double strafe = applyDeadzone(msg->axes[config_.axis_strafe], config_.deadzone);
    double rotate = applyDeadzone(msg->axes[config_.axis_rotate], config_.deadzone);
    
    // 应用速度缩放和最大速度限制
    double speed_scale = getSpeedScaleFactor();
    
    // 构建速度命令
    last_cmd_.linear.x = forward * config_.max_linear_velocity * speed_scale;
    last_cmd_.linear.y = strafe * config_.max_linear_velocity * speed_scale;
    last_cmd_.angular.z = rotate * config_.max_angular_velocity * speed_scale;

    updateArmControl(msg);

    // 立即发布
    cmd_vel_pub_->publish(last_cmd_);
}

double RemoteControlNode::applyDeadzone(double value, double deadzone) {
    // 限制在 [-100, 100] 范围内
    value = std::clamp(value, -100.0, 100.0);
    
    // 应用死区
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    
    // 线性调整：移除死区后进行缩放
    // 如果 value > deadzone，返回 (value - deadzone) / (1 - deadzone)
    // 如果 value < -deadzone，返回 (value + deadzone) / (1 - deadzone)
    if (value > 0) {
        return (value - deadzone) / (1.0 - deadzone);
    } else {
        return (value + deadzone) / (1.0 - deadzone);
    }
}

double RemoteControlNode::getSpeedScaleFactor() {
    return current_speed_scale_;
}

void RemoteControlNode::loadRemoteControlParams(const std::string& config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    YAML::Node params = root["remote_control_node"]["ros__parameters"];
    if (!params) {
        throw std::runtime_error("remote_control_node.ros__parameters 不存在");
    }

    if (params["axis_forward"]) config_.axis_forward = params["axis_forward"].as<int>();
    if (params["axis_strafe"]) config_.axis_strafe = params["axis_strafe"].as<int>();
    if (params["axis_rotate"]) config_.axis_rotate = params["axis_rotate"].as<int>();

    if (params["button_speed_up"]) config_.button_speed_up = params["button_speed_up"].as<int>();
    if (params["button_speed_down"]) config_.button_speed_down = params["button_speed_down"].as<int>();
    if (params["button_stop"]) config_.button_stop = params["button_stop"].as<int>();

    if (params["arm_trigger_axis"]) config_.arm_trigger_axis = params["arm_trigger_axis"].as<int>();
    if (params["arm_trigger_threshold"]) config_.arm_trigger_threshold = params["arm_trigger_threshold"].as<double>();
    if (params["arm_trigger_reset_threshold"]) config_.arm_trigger_reset_threshold = params["arm_trigger_reset_threshold"].as<double>();
    if (params["arm_min_angle_rad"]) config_.arm_min_angle_rad = params["arm_min_angle_rad"].as<double>();
    if (params["arm_max_angle_rad"]) config_.arm_max_angle_rad = params["arm_max_angle_rad"].as<double>();
    if (params["arm_protection_press_limit"]) config_.arm_protection_press_limit = params["arm_protection_press_limit"].as<int>();
    if (params["arm_protection_window_sec"]) config_.arm_protection_window_sec = params["arm_protection_window_sec"].as<double>();

    if (params["max_linear_velocity"]) config_.max_linear_velocity = params["max_linear_velocity"].as<double>();
    if (params["max_angular_velocity"]) config_.max_angular_velocity = params["max_angular_velocity"].as<double>();
    if (params["default_speed_scale"]) config_.default_speed_scale = params["default_speed_scale"].as<double>();
    if (params["speed_scale_step"]) config_.speed_scale_step = params["speed_scale_step"].as<double>();
    if (params["deadzone"]) config_.deadzone = params["deadzone"].as<double>();
    if (params["publish_frequency"]) config_.publish_frequency = params["publish_frequency"].as<double>();
}

double RemoteControlNode::normalizeTrigger(double raw_value) const {
    double normalized = (1.0 - raw_value) / 2.0;
    return std::clamp(normalized, 0.0, 1.0);
}

void RemoteControlNode::readyCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "READY") {
        arm_ready_ = true;
        if (arm_trigger_state_ == ArmTriggerState::COMMAND_SENT) {
            arm_trigger_state_ = ArmTriggerState::WAIT_TRIGGER_RESET;
        }
    }
}

void RemoteControlNode::updateArmControl(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const double trigger = normalizeTrigger(msg->axes[config_.arm_trigger_axis]);
    const bool trigger_active = trigger >= config_.arm_trigger_threshold;
    const bool trigger_reset = trigger <= config_.arm_trigger_reset_threshold;

    if (arm_trigger_state_ == ArmTriggerState::PROTECTED) {
        if (trigger_reset) {
            arm_trigger_state_ = ArmTriggerState::IDLE;
            arm_press_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "delta_arm 触发保护解除");
        }
        return;
    }

    if (arm_trigger_state_ == ArmTriggerState::COMMAND_SENT) {
        if (arm_ready_) {
            arm_trigger_state_ = ArmTriggerState::WAIT_TRIGGER_RESET;
        }
        return;
    }

    if (arm_trigger_state_ == ArmTriggerState::WAIT_TRIGGER_RESET) {
        if (trigger_reset) {
            arm_trigger_state_ = ArmTriggerState::IDLE;
            RCLCPP_DEBUG(this->get_logger(), "delta_arm 扳机已回位，允许再次触发");
        }
        return;
    }

    if (!trigger_active) {
        return;
    }

    auto now = this->now();
    if ((now - last_arm_press_time_).seconds() <= config_.arm_protection_window_sec) {
        arm_press_count_ += 1;
    } else {
        arm_press_count_ = 1;
    }
    last_arm_press_time_ = now;

    if (arm_press_count_ > config_.arm_protection_press_limit) {
        arm_trigger_state_ = ArmTriggerState::PROTECTED;
        RCLCPP_WARN(this->get_logger(),
            "delta_arm 触发过于频繁，进入保护（%d 次 / %.2f s）",
            arm_press_count_, config_.arm_protection_window_sec);
        return;
    }

    const double angle = config_.arm_min_angle_rad +
        (config_.arm_max_angle_rad - config_.arm_min_angle_rad);

    auto arm_msg = motor_control_ros2::msg::ArmTarget();
    arm_msg.header.stamp = this->now();
    arm_msg.target_angles = {angle, angle, angle};
    arm_msg.execute = true;
    arm_target_pub_->publish(arm_msg);

    arm_ready_ = false;
    arm_trigger_state_ = ArmTriggerState::COMMAND_SENT;
    RCLCPP_INFO(this->get_logger(), "发送 delta_arm 目标: %.3f rad", angle);
}

void RemoteControlNode::publishTimer() {
    // 定时发布最后一条命令
    // 这样即使遥控器输入不变，也能持续发送命令信号
    cmd_vel_pub_->publish(last_cmd_);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
