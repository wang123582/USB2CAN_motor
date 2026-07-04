#include "motor_control_ros2/delta_arm_manager_node.hpp"
#include <yaml-cpp/yaml.h>

DeltaArmManager::DeltaArmManager()
    : Node("delta_arm_manager"),
      downward_torque_(0.0),
      landing_timeout_(5.0),
      landing_velocity_threshold_(0.3),
      landing_stable_duration_(0.8),
      landing_kd_(0.05),
      kp_(0.50),
      kd_(0.20),
      max_velocity_(10.0),
      control_frequency_(200.0),
      position_tolerance_(0.05),
      max_position_error_(0.3),
      tracking_error_pause_(3.0),
      angle_diff_tolerance_(0.0873),
      state_(State::INIT),
      max_acceleration_(50.0),
      planner_p_gain_(15.0),
      landing_stability_started_(false),
      ready_published_(false),
      gravity_compensation_torque_(0.8),
      top_idle_timeout_(1.0),
      arm_lower_length_m_(0.160),
      arm_upper_length_m_(0.230),
      gravity_mps2_(9.81),
      strike_plane_drop_m_(0.0),
      strike_timing_offset_s_(0.0),
      strike_delay_override_s_(0.35),
      retract_timeout_s_(0.30),
      tilt_timeout_s_(0.50),
      strike_trigger_pulse_s_(0.05),
      retract_kp_(0.80),
      retract_kd_(0.25),
      retract_torque_ff_(0.0),
      tilt_ready_angle_rad_(0.0),
      tilt_down_angle_rad_(1.0),
      tilt_kp_(0.50),
      tilt_kd_(0.20),
      tilt_torque_ff_(0.0),
      tilt_position_tolerance_(0.05),
      tilt_max_position_error_(0.5),
      tilt_motor_name_("arm_tilt_motor"),
      tilt_position_(0.0),
      tilt_velocity_(0.0),
      tilt_online_(false),
      has_tilt_feedback_(false),
      estimated_launch_height_m_(0.0),
      estimated_fall_time_s_(0.0),
      strike_trigger_sent_(false)
{
  current_positions_.fill(0.0);
  current_velocities_.fill(0.0);
  motors_online_.fill(false);
  has_feedback_.fill(false);
  last_positions_.fill(0.0);
  zero_positions_.fill(0.0);
  target_deltas_rad_.fill(0.0);
  planned_deltas_rad_.fill(0.0);
  current_planned_vels_.fill(0.0);
  motor_max_deltas_.fill(1.567);  // 保守默认值：Motor3 最小实测行程
  motor_names_ = {"arm_delta_motor_1", "arm_delta_motor_2", "arm_delta_motor_3"};

  // 加载配置文件
  try {
    std::string config_file =
        ament_index_cpp::get_package_share_directory("motor_control_ros2") +
        "/config/arm_config.yaml";
    loadConfig(config_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "配置文件加载失败，节点退出: %s", e.what());
    throw;
  }

  // 订阅 ArmTarget 命令
  target_sub_ = this->create_subscription<motor_control_ros2::msg::ArmTarget>(
      "/delta_arm/target", 10,
      std::bind(&DeltaArmManager::armTargetCallback, this, std::placeholders::_1));

  // 订阅 GO8010 电机状态
  state_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&DeltaArmManager::motorStateCallback, this, std::placeholders::_1));

  // 发布 GO8010 电机命令
  cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10);

  // 发布 ready 信号
  ready_pub_ = this->create_publisher<std_msgs::msg::String>("/delta_arm/ready", 10);

  // 触发击球机构
  serve_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/serve/trigger", 10);

  // 创建控制定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DeltaArmManager::controlLoop, this));

  init_start_time_ = this->now();
  launch_time_ = init_start_time_;
  state_enter_time_ = init_start_time_;
  strike_trigger_time_ = init_start_time_;
  RCLCPP_INFO(this->get_logger(),
      "delta_arm_manager 启动，控制频率 %.1f Hz，电机: [%s, %s, %s]",
      control_frequency_,
      motor_names_[0].c_str(), motor_names_[1].c_str(), motor_names_[2].c_str());
  RCLCPP_INFO(this->get_logger(), "状态: INIT → 进入软着陆流程");
}

// ========== 配置加载 ==========

void DeltaArmManager::loadConfig(const std::string& config_file)
{
  YAML::Node cfg = YAML::LoadFile(config_file);

  if (cfg["control_frequency"]) {
    control_frequency_ = cfg["control_frequency"].as<double>();
  }

  auto init = cfg["initialization"];
  if (init) {
    if (init["downward_torque"])          downward_torque_           = init["downward_torque"].as<double>();
    if (init["landing_timeout"])          landing_timeout_           = init["landing_timeout"].as<double>();
    if (init["landing_velocity_threshold"]) landing_velocity_threshold_ = init["landing_velocity_threshold"].as<double>();
    if (init["landing_stable_duration"])  landing_stable_duration_   = init["landing_stable_duration"].as<double>();
    if (init["landing_kd"])              landing_kd_                = init["landing_kd"].as<double>();
    if (init["gravity_compensation_torque"]) gravity_compensation_torque_ = init["gravity_compensation_torque"].as<double>();
  }

  auto pd = cfg["pd"];
  if (pd) {
    if (pd["kp"]) kp_ = pd["kp"].as<double>();
    if (pd["kd"]) kd_ = pd["kd"].as<double>();
  }

  auto mp = cfg["motion_profile"];
  if (mp) {
    if (mp["max_velocity"])     max_velocity_     = mp["max_velocity"].as<double>();
    if (mp["max_acceleration"]) max_acceleration_ = mp["max_acceleration"].as<double>();
    if (mp["planner_p_gain"])   planner_p_gain_   = mp["planner_p_gain"].as<double>();
  }

  auto motors = cfg["motors"];
  if (motors && motors.IsSequence()) {
    for (size_t i = 0; i < 3 && i < motors.size(); ++i) {
      if (motors[i]["name"]) {
        motor_names_[i] = motors[i]["name"].as<std::string>();
      }
      if (motors[i]["max_delta_rad"]) {
        motor_max_deltas_[i] = motors[i]["max_delta_rad"].as<double>();
      }
    }
  }

  if (cfg["position_tolerance"]) {
    position_tolerance_ = cfg["position_tolerance"].as<double>();
  }

  auto safety = cfg["safety"];
  if (safety) {
    if (safety["max_position_error"]) {
      max_position_error_ = safety["max_position_error"].as<double>();
    }
    if (safety["angle_diff_tolerance"]) {
      angle_diff_tolerance_ = safety["angle_diff_tolerance"].as<double>();
    }
  }

  if (cfg["tracking_error_pause"]) {
    tracking_error_pause_ = cfg["tracking_error_pause"].as<double>();
  }

  if (cfg["top_idle_timeout"]) {
    top_idle_timeout_ = cfg["top_idle_timeout"].as<double>();
  }

  auto serve = cfg["serve_test"];
  if (serve) {
    if (serve["arm_lower_length_m"]) arm_lower_length_m_ = serve["arm_lower_length_m"].as<double>();
    if (serve["arm_upper_length_m"]) arm_upper_length_m_ = serve["arm_upper_length_m"].as<double>();
    if (serve["gravity_mps2"]) gravity_mps2_ = serve["gravity_mps2"].as<double>();
    if (serve["strike_plane_drop_m"]) strike_plane_drop_m_ = serve["strike_plane_drop_m"].as<double>();
    if (serve["timing_offset_s"]) strike_timing_offset_s_ = serve["timing_offset_s"].as<double>();
    if (serve["strike_delay_override_s"]) strike_delay_override_s_ = serve["strike_delay_override_s"].as<double>();
    if (serve["retract_timeout_s"]) retract_timeout_s_ = serve["retract_timeout_s"].as<double>();
    if (serve["tilt_timeout_s"]) tilt_timeout_s_ = serve["tilt_timeout_s"].as<double>();
    if (serve["strike_trigger_pulse_s"]) strike_trigger_pulse_s_ = serve["strike_trigger_pulse_s"].as<double>();
  }

  auto retract = cfg["retract"];
  if (retract) {
    if (retract["kp"]) retract_kp_ = retract["kp"].as<double>();
    if (retract["kd"]) retract_kd_ = retract["kd"].as<double>();
    if (retract["torque_ff"]) retract_torque_ff_ = retract["torque_ff"].as<double>();
  }

  auto tilt = cfg["tilt"];
  if (tilt) {
    if (tilt["name"]) tilt_motor_name_ = tilt["name"].as<std::string>();
    if (tilt["ready_angle_rad"]) tilt_ready_angle_rad_ = tilt["ready_angle_rad"].as<double>();
    if (tilt["down_angle_rad"]) tilt_down_angle_rad_ = tilt["down_angle_rad"].as<double>();
    if (tilt["kp"]) tilt_kp_ = tilt["kp"].as<double>();
    if (tilt["kd"]) tilt_kd_ = tilt["kd"].as<double>();
    if (tilt["torque_ff"]) tilt_torque_ff_ = tilt["torque_ff"].as<double>();
    if (tilt["position_tolerance"]) tilt_position_tolerance_ = tilt["position_tolerance"].as<double>();
    if (tilt["max_position_error"]) tilt_max_position_error_ = tilt["max_position_error"].as<double>();
  }

  RCLCPP_INFO(this->get_logger(),
      "配置加载完成: kp=%.2f kd=%.2f max_vel=%.2f max_accel=%.1f p_gain=%.1f landing_torque=%.2f gravity_comp=%.3f tilt=[%s down=%.3f]",
      kp_, kd_, max_velocity_, max_acceleration_, planner_p_gain_, downward_torque_,
      gravity_compensation_torque_, tilt_motor_name_.c_str(), tilt_down_angle_rad_);
}

// ========== 回调 ==========

void DeltaArmManager::armTargetCallback(
    const motor_control_ros2::msg::ArmTarget::SharedPtr msg)
{
  if (!msg->execute) {
    return;
  }

  bool all_feedback_ready = true;
  for (size_t i = 0; i < 3; ++i) {
    if (!has_feedback_[i] || !motors_online_[i]) {
      all_feedback_ready = false;
      break;
    }
  }//判断电机是否在线且反馈就绪，未就绪则拒绝执行命令
  if (!all_feedback_ready) {
    RCLCPP_WARN(this->get_logger(), "收到目标命令但电机反馈未就绪，拒绝执行");
    return;
  }

  if (state_ != State::READY && state_ != State::SOFT_LANDING && state_ != State::INIT) {
    RCLCPP_WARN(this->get_logger(),
        "收到目标命令，但当前状态为 %s，拒绝执行（仅 READY/INIT/SOFT_LANDING 状态接受命令）",
        state_ == State::INIT ? "INIT" :
        state_ == State::SOFT_LANDING ? "SOFT_LANDING" : "UNKNOWN");
    return;
  }

  for (size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(msg->target_angles[i])) {
      RCLCPP_WARN(this->get_logger(), "目标角度存在非法值，拒绝执行");
      return;
    }
  }

  // 三电机目标角允许有小差异（实测行程不完全一致），超过容差则拒绝
  const double a0 = msg->target_angles[0];
  const double a1 = msg->target_angles[1];
  const double a2 = msg->target_angles[2];
  if (std::abs(a0 - a1) > angle_diff_tolerance_ || std::abs(a0 - a2) > angle_diff_tolerance_) {
    RCLCPP_WARN(this->get_logger(),
        "拒绝执行：target_angles 三路差值超限 [%.4f, %.4f, %.4f]，允许最大差值 %.4f rad",
        a0, a1, a2, angle_diff_tolerance_);
    return;
  }

  // 每路独立检查最大行程上限
  for (size_t i = 0; i < 3; ++i) {
    if (msg->target_angles[i] > motor_max_deltas_[i]) {
      RCLCPP_WARN(this->get_logger(),
          "拒绝执行：motor[%zu] 目标 %.4f rad 超过实测上限 %.4f rad",
          i, msg->target_angles[i], motor_max_deltas_[i]);
      return;
    }
  }

  // 若仍在 INIT/SOFT_LANDING，收到 execute 后立即撤去向下力并切入执行
  if (state_ == State::INIT || state_ == State::SOFT_LANDING) {
    for (size_t i = 0; i < 3; ++i) {
      zero_positions_[i] = current_positions_[i];
    }
    landing_stability_started_ = false;
    RCLCPP_WARN(this->get_logger(),
        "收到执行命令，提前结束软着陆并撤去向下力矩，直接进入 EXECUTE");
  }

  for (size_t i = 0; i < 3; ++i) {
    target_deltas_rad_[i]    = msg->target_angles[i];
    planned_deltas_rad_[i]   = 0.0;
    current_planned_vels_[i] = 0.0;
  }
  execute_start_time_ = this->now();
  state_enter_time_ = execute_start_time_;
  strike_trigger_sent_ = false;
  const double avg_target = (a0 + a1 + a2) / 3.0;
  estimated_launch_height_m_ = estimateLaunchHeight(avg_target);
  estimated_fall_time_s_ = estimateFallTime(estimated_launch_height_m_);
  state_ = State::EXECUTE;
  RCLCPP_INFO(this->get_logger(),
      "READY → EXECUTE: 目标增量 [%.3f, %.3f, %.3f] rad；估算高度 %.3f m，下落时间 %.3f s",
      target_deltas_rad_[0], target_deltas_rad_[1], target_deltas_rad_[2],
      estimated_launch_height_m_, estimated_fall_time_s_);
}

void DeltaArmManager::motorStateCallback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
  for (size_t i = 0; i < 3; ++i) {
    if (msg->joint_name == motor_names_[i]) {
      current_positions_[i]  = static_cast<double>(msg->position);
      current_velocities_[i] = static_cast<double>(msg->velocity);
      motors_online_[i]      = msg->online;
      has_feedback_[i]       = true;
      break;
    }
  }

  if (msg->joint_name == tilt_motor_name_) {
    tilt_position_ = static_cast<double>(msg->position);
    tilt_velocity_ = static_cast<double>(msg->velocity);
    tilt_online_ = msg->online;
    has_tilt_feedback_ = true;
  }
}

// ========== 主控制循环 ==========

void DeltaArmManager::controlLoop()
{
  const double dt = 1.0 / control_frequency_;

  switch (state_) {

    case State::INIT: {
      // 进入软着陆阶段
      state_ = State::SOFT_LANDING;
      landing_stability_started_ = false;
      init_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "INIT → SOFT_LANDING: 施加向下力矩 %.2f Nm + 阻尼 kd=%.3f", downward_torque_, landing_kd_);
      break;
    }

    case State::SOFT_LANDING: {
      // 检查超时
      double elapsed = (this->now() - init_start_time_).seconds();
      if (elapsed > landing_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "软着陆超时 (%.1f s)，强制进入 READY 状态", landing_timeout_);
        state_ = State::READY;
        publishReady();
        break;
      }

      // 向所有电机施加向下力矩 + 阻尼限速
      // τ = τ_ff + Kd × (0 - ω)：力矩驱动下降，阻尼自动限速
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, 0.0, 0.0, downward_torque_, 0.0, landing_kd_);
      }

      // 检查着陆稳定条件（速度反馈 < 阈值）
      if (allMotorsLanded()) {
        if (!landing_stability_started_) {
          landing_stable_since_ = this->now();
          landing_stability_started_ = true;
        }
        double stable_time = (this->now() - landing_stable_since_).seconds();
        if (stable_time >= landing_stable_duration_) {
          RCLCPP_INFO(this->get_logger(),
              "软着陆完成（稳定 %.2f s），SOFT_LANDING → READY", stable_time);
          // 解耦：将当前物理角锁定为零点，后续控制坐标从 0 rad 开始
          for (size_t i = 0; i < 3; ++i) {
            zero_positions_[i] = current_positions_[i];
          }
          planned_deltas_rad_.fill(0.0);
          target_deltas_rad_.fill(0.0);
          current_planned_vels_.fill(0.0);
          state_ = State::READY;
          publishReady();
        }
      } else {
        landing_stability_started_ = false;
      }
      break;
    }

    case State::READY: {
      // 发布 READY 信号（每秒一次，防止订阅者错过）
      if (!ready_published_) {
        publishReady();
      }
      /*  在 READY 阶段保持解耦零点（target_delta=0）+ 重力补偿前馈
        且每次执行完后会主动回到零点    */
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, 0, kp_, kd_);
      }
      publishTiltCommand(tilt_ready_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);
      break;
    }

    case State::EXECUTE: {
      // 上抛超时保护：测试阶段保留，避免异常时卡在 EXECUTE。
      double execute_elapsed = (this->now() - execute_start_time_).seconds();
      if (execute_elapsed > top_idle_timeout_) {
        RCLCPP_WARN(this->get_logger(), "EXECUTE 超时 (%.1f s)，强制进入 FAST_RETRACT", top_idle_timeout_);
        enterFastRetract();
        break;
      }

      // 自适应规划：任一电机跟不上则全路暂停
      bool tracking_ok = true;
      for (size_t i = 0; i < 3; ++i) {
        if (has_feedback_[i]) {
          double planned_physical = zero_positions_[i] + planned_deltas_rad_[i];
          double tracking_err = std::abs(planned_physical - current_positions_[i]);
          if (tracking_err > tracking_error_pause_) {
            tracking_ok = false;
            break;
          }
        }
      }

      // 平滑轨迹生成：每路独立规划（目标行程略有差异）
      const double max_dv = max_acceleration_ * dt;
      for (size_t i = 0; i < 3; ++i) {
        if (tracking_ok) {
          double err = target_deltas_rad_[i] - planned_deltas_rad_[i];
          double target_vel = std::clamp(err * planner_p_gain_, -max_velocity_, max_velocity_);
          if (target_vel > current_planned_vels_[i] + max_dv) {
            current_planned_vels_[i] += max_dv;
          } else if (target_vel < current_planned_vels_[i] - max_dv) {
            current_planned_vels_[i] -= max_dv;
          } else {
            current_planned_vels_[i] = target_vel;
          }
          planned_deltas_rad_[i] += current_planned_vels_[i] * dt;
        } else {
          current_planned_vels_[i] = 0.0;
        }
        const double physical_target = zero_positions_[i] + planned_deltas_rad_[i];
        publishCommand(i, physical_target, current_planned_vels_[i],
                       gravity_compensation_torque_, kp_, kd_);
      }

      if (allMotorsReached()) {
        RCLCPP_INFO(this->get_logger(), "上抛目标到达，EXECUTE → FAST_RETRACT");
        enterFastRetract();
      }
      break;
    }

    case State::FAST_RETRACT: {
      // 快速收拍：直接发布相对 0 度，不再用慢速下降规划。
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, retract_torque_ff_, retract_kp_, retract_kd_);
      }
      publishTiltCommand(tilt_down_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);

      const double elapsed = (this->now() - state_enter_time_).seconds();
      if (allMotorsAtZero() || elapsed >= retract_timeout_s_) {
        RCLCPP_INFO(this->get_logger(),
            "FAST_RETRACT → TILT_DOWN: retracted=%s elapsed=%.3f",
            allMotorsAtZero() ? "true" : "false", elapsed);
        state_ = State::TILT_DOWN;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::TILT_DOWN: {
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, retract_torque_ff_, retract_kp_, retract_kd_);
      }
      publishTiltCommand(tilt_down_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);

      const double elapsed = (this->now() - state_enter_time_).seconds();
      if (tiltReached(tilt_down_angle_rad_) || elapsed >= tilt_timeout_s_) {
        const double modeled_delay = std::max(0.0, estimated_fall_time_s_ + strike_timing_offset_s_);
        const double strike_delay = strike_delay_override_s_ >= 0.0 ? strike_delay_override_s_ : modeled_delay;
        RCLCPP_INFO(this->get_logger(),
            "TILT_DOWN → WAIT_STRIKE: tilt_reached=%s elapsed=%.3f strike_delay=%.3f modeled=%.3f",
            tiltReached(tilt_down_angle_rad_) ? "true" : "false",
            elapsed, strike_delay, modeled_delay);
        state_ = State::WAIT_STRIKE;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::WAIT_STRIKE: {
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, retract_torque_ff_, retract_kp_, retract_kd_);
      }
      publishTiltCommand(tilt_down_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);

      const double modeled_delay = std::max(0.0, estimated_fall_time_s_ + strike_timing_offset_s_);
      const double strike_delay = strike_delay_override_s_ >= 0.0 ? strike_delay_override_s_ : modeled_delay;
      // elapsed 从 launch_time_（上抛完成瞬间）算起，而非从进入 WAIT_STRIKE 起。
      // 若 retract+tilt 总耗时已超过 strike_delay，进入本状态后立即触发——这是预期行为。
      // 若需要更长延时，将 strike_delay_override_s_ 设为 retract_timeout_s_ + tilt_timeout_s_ 以上。
      const double elapsed_since_launch = (this->now() - launch_time_).seconds();
      if (elapsed_since_launch >= strike_delay) {
        state_ = State::TRIGGER_STRIKE;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::TRIGGER_STRIKE: {
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, retract_torque_ff_, retract_kp_, retract_kd_);
      }
      publishTiltCommand(tilt_down_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);

      auto trigger = std_msgs::msg::Bool();
      trigger.data = true;
      serve_trigger_pub_->publish(trigger);

      if (!strike_trigger_sent_) {
        strike_trigger_sent_ = true;
        strike_trigger_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "已触发 /serve/trigger，等待 %.3f s 后恢复 READY", strike_trigger_pulse_s_);
      }

      if ((this->now() - strike_trigger_time_).seconds() >= strike_trigger_pulse_s_) {
        state_ = State::RECOVER_READY;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::RECOVER_READY: {
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, 0.0, kp_, kd_);
      }
      publishTiltCommand(tilt_ready_angle_rad_, 0.0, tilt_torque_ff_, tilt_kp_, tilt_kd_);
      planned_deltas_rad_.fill(0.0);
      current_planned_vels_.fill(0.0);
      target_deltas_rad_.fill(0.0);
      ready_published_ = false;
      state_ = State::READY;
      state_enter_time_ = this->now();
      break;
    }
  }
}

// ========== 辅助函数 ==========

bool DeltaArmManager::allMotorsLanded() const
{
  // 使用速度反馈判定着陆（速度由电机内部高频计算，不受 ROS 反馈频率影响）
  for (size_t i = 0; i < 3; ++i) {
    if (!motors_online_[i] || !has_feedback_[i]) {
      return false;
    }
    if (std::abs(current_velocities_[i]) >= landing_velocity_threshold_) {
      return false;
    }
  }
  return true;
}

bool DeltaArmManager::allMotorsReached() const
{
  for (size_t i = 0; i < 3; ++i) {
    if (!motors_online_[i] || !has_feedback_[i]) {
      return false;
    }
    const double logical_pos = current_positions_[i] - zero_positions_[i];
    if (std::abs(target_deltas_rad_[i] - logical_pos) > position_tolerance_) {
      return false;
    }
  }
  return true;
}

bool DeltaArmManager::allMotorsAtZero() const
{
  for (size_t i = 0; i < 3; ++i) {
    if (!motors_online_[i] || !has_feedback_[i]) {
      return false;
    }
    const double logical_pos = current_positions_[i] - zero_positions_[i];
    if (std::abs(logical_pos) > position_tolerance_) {
      return false;
    }
  }
  return true;
}

bool DeltaArmManager::tiltReached(double target_rad) const
{
  if (!has_tilt_feedback_ || !tilt_online_) {
    return false;
  }
  return std::abs(target_rad - tilt_position_) <= tilt_position_tolerance_;
}

double DeltaArmManager::estimateLaunchHeight(double delta_rad) const
{
  // 测试估算：先把上下臂看作等效直杆，只用于录像对比，不参与控制判定。
  const double equivalent_length = arm_lower_length_m_ + arm_upper_length_m_;
  return std::max(0.0, equivalent_length * std::sin(std::abs(delta_rad)));
}

double DeltaArmManager::estimateFallTime(double height_m) const
{
  if (gravity_mps2_ <= 0.0) {
    return 0.0;
  }
  const double launch_v = std::sqrt(std::max(0.0, 2.0 * gravity_mps2_ * height_m));
  const double drop = std::max(0.0, strike_plane_drop_m_);
  return (launch_v + std::sqrt(launch_v * launch_v + 2.0 * gravity_mps2_ * drop)) / gravity_mps2_;
}

void DeltaArmManager::enterFastRetract()
{
  launch_time_ = this->now();
  state_enter_time_ = launch_time_;
  planned_deltas_rad_.fill(0.0);
  current_planned_vels_.fill(0.0);
  target_deltas_rad_.fill(0.0);
  state_ = State::FAST_RETRACT;

  RCLCPP_INFO(this->get_logger(),
      "进入 FAST_RETRACT：三路上抛电机立即回相对 0；测试估算高度 %.3f m，下落时间 %.3f s",
      estimated_launch_height_m_, estimated_fall_time_s_);
}

void DeltaArmManager::publishReady()
{
  auto msg = std_msgs::msg::String();
  msg.data = "READY";
  ready_pub_->publish(msg);
  ready_published_ = true;
  RCLCPP_INFO(this->get_logger(), "发布 /delta_arm/ready: READY");
}

void DeltaArmManager::publishCommand(size_t idx,
    double pos_des, double vel_des, double torque_ff,
    double kp, double kd)
{
  // 位置误差钳位：防止大误差产生过大力矩导致振荡
  double clamped_pos = pos_des;
  if (has_feedback_[idx] && max_position_error_ > 0.0) {
    double error = pos_des - current_positions_[idx];
    if (std::abs(error) > max_position_error_) {
      clamped_pos = current_positions_[idx] + std::copysign(max_position_error_, error);
    }
  }

  
  double clamped_torque_ff = torque_ff;

  auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
  cmd.header.stamp = this->now();
  cmd.joint_name = motor_names_[idx];
  cmd.mode = motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC;
  cmd.position_target = clamped_pos;
  cmd.velocity_target = vel_des;
  cmd.torque_ff       = static_cast<float>(clamped_torque_ff);
  cmd.kp              = static_cast<float>(kp);
  cmd.kd              = static_cast<float>(kd);
  cmd_pub_->publish(cmd);
}

void DeltaArmManager::publishTiltCommand(double pos_des, double vel_des, double torque_ff,
    double kp, double kd)
{
  double clamped_pos = pos_des;
  if (has_tilt_feedback_ && tilt_max_position_error_ > 0.0) {
    const double error = pos_des - tilt_position_;
    if (std::abs(error) > tilt_max_position_error_) {
      clamped_pos = tilt_position_ + std::copysign(tilt_max_position_error_, error);
    }
  }

  auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
  cmd.header.stamp = this->now();
  cmd.joint_name = tilt_motor_name_;
  cmd.mode = motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC;
  cmd.position_target = clamped_pos;
  cmd.velocity_target = vel_des;
  cmd.torque_ff = static_cast<float>(torque_ff);
  cmd.kp = static_cast<float>(kp);
  cmd.kd = static_cast<float>(kd);
  cmd_pub_->publish(cmd);
}

// ========== main ==========

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
