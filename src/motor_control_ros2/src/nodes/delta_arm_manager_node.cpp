#include "motor_control_ros2/delta_arm_manager_node.hpp"
#include <yaml-cpp/yaml.h>

DeltaArmManager::DeltaArmManager()
    : Node("delta_arm_manager"),
      downward_torque_(0.0),
      landing_timeout_(5.0),
      landing_velocity_threshold_(0.3),
      landing_stable_duration_(0.8),
      landing_kd_(0.05),
      landing_debug_log_(true),
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
      top_press_margin_rad_(0.10),
      top_approach_band_rad_(0.20),
      stop_settle_vel_(0.5),
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
      retract_kp_(1.50),
      retract_kd_(0.15),
      retract_torque_ff_(-0.5),
      retract_max_velocity_(18.0),
      retract_max_acceleration_(130.0),
      retract_bottom_soft_(0.35),
      retract_debug_log_(true),
      tilt_ready_angle_rad_(0.0),
      tilt_down_angle_rad_(1.0),
      tilt_kp_(2.0),
      tilt_kd_(0.30),
      tilt_hold_ff_(1.1),
      tilt_rate_rad_s_(6.0),
      tilt_position_tolerance_(0.05),
      tilt_max_position_error_(0.5),
      tilt_motor_name_("arm_tilt_motor"),
      tilt_position_(0.0),
      tilt_velocity_(0.0),
      tilt_online_(false),
      has_tilt_feedback_(false),
      tilt_zero_position_(0.0),
      tilt_zero_captured_(false),
      tilt_cmd_angle_(0.0),
      estimated_launch_height_m_(0.0),
      estimated_fall_time_s_(0.0),
      strike_trigger_sent_(false),
      wind_status_(""),
      wind_catch_seen_(false),
      wind_done_timeout_s_(8.0)
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

  // 订阅 windmill 状态：等它打完整循环(CATCH 完毕)俯仰再回摆
  wind_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/serve/status", 10,
      std::bind(&DeltaArmManager::windStatusCallback, this, std::placeholders::_1));

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
  RCLCPP_INFO(this->get_logger(), "状态：初始化 → 进入软着陆流程");
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
    if (init["debug_log"]) landing_debug_log_ = init["debug_log"].as<bool>();
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
    if (mp["top_press_margin_rad"])  top_press_margin_rad_  = mp["top_press_margin_rad"].as<double>();
    if (mp["top_approach_band_rad"]) top_approach_band_rad_ = mp["top_approach_band_rad"].as<double>();
    if (mp["stop_settle_vel"])       stop_settle_vel_       = mp["stop_settle_vel"].as<double>();
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
    if (serve["wind_done_timeout_s"]) wind_done_timeout_s_ = serve["wind_done_timeout_s"].as<double>();
  }

  auto retract = cfg["retract"];
  if (retract) {
    if (retract["kp"]) retract_kp_ = retract["kp"].as<double>();
    if (retract["kd"]) retract_kd_ = retract["kd"].as<double>();
    if (retract["torque_ff"]) retract_torque_ff_ = retract["torque_ff"].as<double>();
    if (retract["max_velocity"]) retract_max_velocity_ = retract["max_velocity"].as<double>();
    if (retract["max_acceleration"]) retract_max_acceleration_ = retract["max_acceleration"].as<double>();
    if (retract["bottom_soft_rad"]) retract_bottom_soft_ = retract["bottom_soft_rad"].as<double>();
    if (retract["debug_log"]) retract_debug_log_ = retract["debug_log"].as<bool>();
  }

  auto tilt = cfg["tilt"];
  if (tilt) {
    if (tilt["name"]) tilt_motor_name_ = tilt["name"].as<std::string>();
    if (tilt["ready_angle_rad"]) tilt_ready_angle_rad_ = tilt["ready_angle_rad"].as<double>();
    if (tilt["down_angle_rad"]) tilt_down_angle_rad_ = tilt["down_angle_rad"].as<double>();
    if (tilt["kp"]) tilt_kp_ = tilt["kp"].as<double>();
    if (tilt["kd"]) tilt_kd_ = tilt["kd"].as<double>();
    if (tilt["hold_torque_ff"]) tilt_hold_ff_ = tilt["hold_torque_ff"].as<double>();
    if (tilt["rate_rad_s"]) tilt_rate_rad_s_ = tilt["rate_rad_s"].as<double>();
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
        "收到目标命令，但当前状态为 %s，拒绝执行（仅就绪/初始化/软着陆状态接受命令）",
        state_ == State::INIT ? "初始化" :
        state_ == State::SOFT_LANDING ? "软着陆" : "未知");
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
        "收到执行命令，提前结束软着陆并撤去向下力矩，直接进入上抛阶段");
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
      "就绪 → 上抛：目标增量 [%.3f, %.3f, %.3f] rad，估算高度 %.3f m，下落时间 %.3f s",
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
    // 俯仰零点解耦：GO8010 反馈是电机上电坐标系，首帧在线反馈时锁定当前物理角为零点，
    // 之后 ready/down 角全部相对该零点（与三路 delta 的 zero_positions_ 同理）。
    if (!tilt_zero_captured_ && msg->online) {
      tilt_zero_position_ = tilt_position_;
      tilt_cmd_angle_ = 0.0;
      tilt_zero_captured_ = true;
      RCLCPP_INFO(this->get_logger(),
          "俯仰零点锁定：原始角 %.3f rad，此后 ready=%.3f down=%.3f 均为相对角",
          tilt_zero_position_, tilt_ready_angle_rad_, tilt_down_angle_rad_);
    }
  }
}

void DeltaArmManager::windStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // 只缓存最新 windmill 状态名，WAIT_WIND_DONE 里据此判断"接住完毕"
  wind_status_ = msg->data;
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
      RCLCPP_INFO(this->get_logger(), "初始化 → 软着陆：施加向下力矩 %.2f Nm，阻尼 kd=%.3f", downward_torque_, landing_kd_);
      // 定位"刚开始的向下力"：打印 CSV 表头，后续每周期一行。
      //   cmd_tq=下发给三路 delta 的前馈力矩(=downward_torque)  landing_kd=阻尼
      //   m{i}_pos/vel=各 delta 反馈角/速度(原始上电系)  tilt_pos/vel=俯仰反馈
      // 判读：cmd_tq≈0 但 m_vel 为负持续下沉 → 自重(重力)；tilt_vel 非零 → 俯仰在动。
      if (landing_debug_log_) {
        RCLCPP_INFO(this->get_logger(),
            "LANDING_CSV,t,cmd_tq,landing_kd,"
            "m1_pos,m1_vel,m2_pos,m2_vel,m3_pos,m3_vel,tilt_pos,tilt_vel");
      }
      break;
    }

    case State::SOFT_LANDING: {
      // 检查超时
      double elapsed = (this->now() - init_start_time_).seconds();
      if (elapsed > landing_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "软着陆超时 (%.1f s)，强制进入 READY 状态", landing_timeout_);
        // 超时路径同样要锁定零点：否则 READY 会拿默认 0（上电原点）当零点硬拉三路电机
        for (size_t i = 0; i < 3; ++i) {
          zero_positions_[i] = current_positions_[i];
        }
        planned_deltas_rad_.fill(0.0);
        target_deltas_rad_.fill(0.0);
        current_planned_vels_.fill(0.0);
        state_ = State::READY;
        publishReady();
        break;
      }

      // 向所有电机施加向下力矩 + 阻尼限速
      // τ = τ_ff + Kd × (0 - ω)：力矩驱动下降，阻尼自动限速
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, 0.0, 0.0, downward_torque_, 0.0, landing_kd_);
      }

      // 软着陆逐周期数据记录：对齐 INIT 打印的表头
      if (landing_debug_log_) {
        RCLCPP_INFO(this->get_logger(),
            "LANDING_CSV,%.4f,%.3f,%.3f,"
            "%.4f,%.3f,%.4f,%.3f,%.4f,%.3f,%.4f,%.3f",
            elapsed, downward_torque_, landing_kd_,
            current_positions_[0], current_velocities_[0],
            current_positions_[1], current_velocities_[1],
            current_positions_[2], current_velocities_[2],
            tilt_position_, tilt_velocity_);
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
              "软着陆完成（稳定 %.2f s），软着陆 → 就绪", stable_time);
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
      tiltCommand(tilt_ready_angle_rad_, dt);
      break;
    }

    case State::EXECUTE: {
      // 上抛超时保护：测试阶段保留，避免异常时卡在 EXECUTE。
      double execute_elapsed = (this->now() - execute_start_time_).seconds();
      if (execute_elapsed > top_idle_timeout_) {
        RCLCPP_WARN(this->get_logger(), "上抛阶段超时 (%.1f s)，强制进入快速收拍", top_idle_timeout_);
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

      // 冲顶部限位：三路满速上抛 → 接近顶部降速软压 → 都被挡块顶到同位置、速度一起归零。
      // 目标压到限位再往上 top_press_margin，PD 持续把臂顶向挡块（机械挡块负责三路对齐+反力）。
      const double max_dv = max_acceleration_ * dt;
      for (size_t i = 0; i < 3; ++i) {
        if (tracking_ok) {
          // 球在顶部靠机械臂撞挡块急停才脱手 → 出手速度 = 撞挡块前的臂速，必须最大、绝不减速。
          // 目标越过限位一点(press_margin)，让规划全程满加速冲向挡块、不进减速段（挡块吃冲击+同步）。
          const double press_target = target_deltas_rad_[i] + top_press_margin_rad_;
          double err = press_target - planned_deltas_rad_[i];
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
        // 上抛同样关闭位置钳位（bypass_clamp=true）：速度优先，让 PD 看到完整误差、
        // 电机落后时也能跑满上抛力矩。粗跟踪失控仍由 tracking_error_pause_ 兜底。
        publishCommand(i, physical_target, current_planned_vels_[i],
                       gravity_compensation_torque_, kp_, kd_, /*bypass_clamp=*/true);
      }

      // 上抛期间俯仰保持待机位：恒定向上前馈托住 + 单一 PD 锁在待机角，
      // 抵抗三路上抛反作用力矩摇动减速箱（齿隙撞击/异响）。
      tiltCommand(tilt_ready_angle_rad_, dt);

      // 撞停判据（不再用 allMotorsReached，只看位置会因三路到达时间不同而 +/- 割裂）：
      // 三路都已顶到接近限位处（ad 进入 approach_band）且实际速度都降到阈值以下（被挡块顶停、
      // 一起到 0）→ 三路在挡块上对齐，此刻进收拍就是从同步态一起往下走。
      bool all_settled = true;
      for (size_t i = 0; i < 3; ++i) {
        if (!motors_online_[i] || !has_feedback_[i]) { all_settled = false; break; }
        const double ad = current_positions_[i] - zero_positions_[i];
        if (ad < target_deltas_rad_[i] - top_approach_band_rad_) { all_settled = false; break; }
        if (std::abs(current_velocities_[i]) > stop_settle_vel_) { all_settled = false; break; }
      }
      if (all_settled) {
        RCLCPP_INFO(this->get_logger(), "三路撞停顶部限位对齐 → 快速收拍");
        enterFastRetract();
      }
      break;
    }

    case State::FAST_RETRACT: {
      // 快速收拍（简化版）：顶部限位已把三路对齐（同位置、同零速），从同步态一起往下走。
      // 三路共享一条开环梯形轨迹直接回 0，不再做 lead_cap 牵引 / 滞后预测那套——因为起点已同步，
      // 三路跟同一条命令下降，不会再出现"有的正有的负"；顺滑非重点，开环反而无牵引锯齿。
      // 仅保留底部软着陆：贴零前撤下向下前馈，防带速砸底反弹再次产生 +/- 割裂。
      const double retract_max_dv = retract_max_acceleration_ * dt;

      // 共享规划角/速度（复用 [0] 存共享量）：朝 0 的开环梯形，距零 max_v/p_gain 内开始减速
      double shared_planned = planned_deltas_rad_[0];
      double shared_vel     = current_planned_vels_[0];
      const double target_vel = std::clamp((0.0 - shared_planned) * planner_p_gain_,
                                           -retract_max_velocity_, retract_max_velocity_);
      if (target_vel > shared_vel + retract_max_dv)      shared_vel += retract_max_dv;
      else if (target_vel < shared_vel - retract_max_dv) shared_vel -= retract_max_dv;
      else                                               shared_vel = target_vel;
      shared_planned += shared_vel * dt;
      // 不冲过零点：规划角不低于 0，避免把电机往机械底下面硬拉
      if (shared_planned < 0.0) { shared_planned = 0.0; if (shared_vel < 0.0) shared_vel = 0.0; }

      for (size_t i = 0; i < 3; ++i) {
        planned_deltas_rad_[i]   = shared_planned;   // 三路同一条轨迹
        current_planned_vels_[i] = shared_vel;
        const double physical_target = zero_positions_[i] + shared_planned;
        // 底部软着陆：实际角低于 bottom_soft 时撤下向下前馈，先到的电机贴零点等其他两路
        const double ad = current_positions_[i] - zero_positions_[i];
        const double ff = (ad < retract_bottom_soft_) ? 0.0 : retract_torque_ff_;
        publishCommand(i, physical_target, shared_vel, ff, retract_kp_, retract_kd_, /*bypass_clamp=*/true);
      }
      tiltCommand(tilt_down_angle_rad_, dt);
      maybeTriggerStrike();  // 挥拍时机只看 launch 后 strike_delay，收拍途中到点就发，不等收拍完

      const double elapsed = (this->now() - state_enter_time_).seconds();

      // 收拍逐周期数据记录：每字段对齐 enterFastRetract 打印的表头
      if (retract_debug_log_) {
        const double ad0 = current_positions_[0] - zero_positions_[0];
        const double ad1 = current_positions_[1] - zero_positions_[1];
        const double ad2 = current_positions_[2] - zero_positions_[2];
        const double tilt_ad = tilt_zero_captured_ ? (tilt_position_ - tilt_zero_position_) : 0.0;
        RCLCPP_INFO(this->get_logger(),
            "RETRACT_CSV,%.4f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,"
            "%.4f,%.4f,%.3f",
            elapsed,
            planned_deltas_rad_[0], ad0, planned_deltas_rad_[0] - ad0, current_planned_vels_[0], current_velocities_[0],
            planned_deltas_rad_[1], ad1, planned_deltas_rad_[1] - ad1, current_planned_vels_[1], current_velocities_[1],
            planned_deltas_rad_[2], ad2, planned_deltas_rad_[2] - ad2, current_planned_vels_[2], current_velocities_[2],
            tilt_cmd_angle_, tilt_ad, tilt_velocity_);
      }

      if (allMotorsAtZero() || elapsed >= retract_timeout_s_) {
        RCLCPP_INFO(this->get_logger(),
            "快速收拍 → 俯仰下压：已回零=%s 耗时=%.3f s",
            allMotorsAtZero() ? "是" : "超时", elapsed);
        state_ = State::TILT_DOWN;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::TILT_DOWN: {
      // 三路锁零点、俯仰抬到击球位；挥拍(windmill 开火)由 maybeTriggerStrike 按 launch 计时独立发。
      // 一旦已发过开火（挥拍时机到），立即进 WAIT_WIND_DONE 等 windmill 打完——不再被抬俯仰时长卡住。
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, retract_torque_ff_, retract_kp_, retract_kd_);
      }
      tiltCommand(tilt_down_angle_rad_, dt);

      const bool fired = maybeTriggerStrike();
      const double elapsed = (this->now() - state_enter_time_).seconds();
      if (fired || elapsed >= tilt_timeout_s_) {
        if (!strike_trigger_sent_) { maybeTriggerStrike(/*force=*/true); }  // 抬俯仰超时兜底强发
        state_ = State::WAIT_WIND_DONE;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::WAIT_WIND_DONE: {
      // 保持三路在零点、俯仰在击球位，等 windmill 打完接住 + 重力归零全部完成再回摆。
      // windmill 击球后循环：FIRE → FREE_WHEEL → CATCH → GRAVITY_HOMING → WIND_UP。
      // "完毕" = 观察到 CATCH（接住）后，windmill 完成重力归零、重新引拍进入 WIND_UP。
      // 用 WIND_UP 而非"离开 CATCH"做判据：离开 CATCH 只是进入 GRAVITY_HOMING（重力归零刚开始），
      // 此时归零尚未完成；WIND_UP 才代表重力归零已完成（handleGravityHoming 归零完才 → WIND_UP）。
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, 0.0, kp_, kd_);
      }
      tiltCommand(tilt_down_angle_rad_, dt);

      if (wind_status_ == "CATCH") {
        wind_catch_seen_ = true;
      }
      const bool wind_done = wind_catch_seen_ && wind_status_ == "WIND_UP";
      const double elapsed = (this->now() - state_enter_time_).seconds();
      if (wind_done || elapsed >= wind_done_timeout_s_) {
        RCLCPP_INFO(this->get_logger(),
            "windmill 完毕(%s) → 俯仰回摆", wind_done ? "接住+重力归零完成" : "超时兜底");
        state_ = State::RECOVER_READY;
        state_enter_time_ = this->now();
      }
      break;
    }

    case State::RECOVER_READY: {
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, 0.0, kp_, kd_);
      }
      tiltCommand(tilt_ready_angle_rad_, dt);
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
  if (!has_tilt_feedback_ || !tilt_online_ || !tilt_zero_captured_) {
    return false;
  }
  const double logical_pos = tilt_position_ - tilt_zero_position_;
  return std::abs(target_rad - logical_pos) <= tilt_position_tolerance_;
}

bool DeltaArmManager::maybeTriggerStrike(bool force)
{
  if (strike_trigger_sent_) {
    return true;
  }
  const double modeled_delay = std::max(0.0, estimated_fall_time_s_ + strike_timing_offset_s_);
  const double strike_delay = strike_delay_override_s_ >= 0.0 ? strike_delay_override_s_ : modeled_delay;
  const double elapsed = (this->now() - launch_time_).seconds();
  if (!force && elapsed < strike_delay) {
    return false;
  }
  auto trigger = std_msgs::msg::Bool();
  trigger.data = true;
  serve_trigger_pub_->publish(trigger);
  strike_trigger_sent_ = true;
  strike_trigger_time_ = this->now();
  wind_catch_seen_ = false;  // 复位本次发球的 CATCH 观测
  RCLCPP_INFO(this->get_logger(),
      "已触发 /serve/trigger（挥拍），launch 后 %.3f s（strike_delay=%.3f 模型=%.3f）",
      elapsed, strike_delay, modeled_delay);
  return true;
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
  // 三路同步收拍：共享规划轨迹从"最高（最落后）那路"的实际位置起步，速度前馈从 0 爬升。
  // 用最高者做起点，避免一进收拍就把已经较低的电机往上提。
  double start_actual = 0.0;
  bool any_fb = false;
  for (size_t i = 0; i < 3; ++i) {
    if (has_feedback_[i]) {
      const double ad = current_positions_[i] - zero_positions_[i];
      if (!any_fb || ad > start_actual) { start_actual = ad; any_fb = true; }
    }
  }
  if (!any_fb) start_actual = target_deltas_rad_[0];
  for (size_t i = 0; i < 3; ++i) {
    planned_deltas_rad_[i] = start_actual;   // [0] 作共享量，三路一致
    current_planned_vels_[i] = 0.0;
    target_deltas_rad_[i] = 0.0;
  }
  state_ = State::FAST_RETRACT;

  RCLCPP_INFO(this->get_logger(),
      "进入快速收拍：三路电机回零点，估算高度 %.3f m，下落时间 %.3f s",
      estimated_launch_height_m_, estimated_fall_time_s_);

  // 收拍卡顿定位：打印 CSV 表头，后续每周期一行。用法：
  //   ros2 run ... delta_arm_manager 2>&1 | grep RETRACT_CSV > retract.csv
  // 字段：pd=规划增量 ad=实际增量 terr=跟踪误差(pd-ad) pv=规划速度 av=实际速度
  //       tilt_cmd/pos/vel=俯仰命令角/反馈角/反馈速度；卡顿时刻看 terr 突增(电机跟不上)
  //       还是 tilt_vel 峰值同步(俯仰耦合)。
  if (retract_debug_log_) {
    RCLCPP_INFO(this->get_logger(),
        "RETRACT_CSV,t,"
        "m1_pd,m1_ad,m1_terr,m1_pv,m1_av,"
        "m2_pd,m2_ad,m2_terr,m2_pv,m2_av,"
        "m3_pd,m3_ad,m3_terr,m3_pv,m3_av,"
        "tilt_cmd,tilt_pos,tilt_vel");
  }
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
    double kp, double kd, bool bypass_clamp)
{
  // 位置误差钳位：防止大误差产生过大力矩导致振荡。
  // 收拍(bypass_clamp)时关闭钳位：速度优先，让 PD 看到完整误差、跑满回落力矩。
  double clamped_pos = pos_des;
  if (!bypass_clamp && has_feedback_[idx] && max_position_error_ > 0.0) {
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

void DeltaArmManager::tiltCommand(double target_rel, double dt)
{
  // 简化俯仰：命令角按单一 rate 限速滑向目标角 + 恒定向上前馈托住臂自重 + 单一 PD。
  // 一直挂 tilt_hold_ff_（向上）抵消臂重后，抬起和回摆对称，不再分 raise/return 两套逻辑。
  const double step = tilt_rate_rad_s_ * dt;
  const double err = target_rel - tilt_cmd_angle_;
  double cmd_vel;
  if (std::abs(err) <= step) {
    tilt_cmd_angle_ = target_rel;
    cmd_vel = 0.0;
  } else {
    tilt_cmd_angle_ += std::copysign(step, err);
    cmd_vel = std::copysign(tilt_rate_rad_s_, err);  // 速度前馈随滑动方向
  }
  publishTiltCommand(tilt_cmd_angle_, cmd_vel, tilt_hold_ff_, tilt_kp_, tilt_kd_);
}

void DeltaArmManager::publishTiltCommand(double angle_rel, double vel_des, double torque_ff,
    double kp, double kd)
{
  // 零点未锁定前不发俯仰命令：此时坐标系未知，发 0.0 会把电机往上电原点硬拉（异响根源）
  if (!tilt_zero_captured_) {
    return;
  }
  const double pos_des = tilt_zero_position_ + angle_rel;
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
