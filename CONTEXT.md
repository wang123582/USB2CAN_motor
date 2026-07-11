# 项目上下文 (CONTEXT.md)
> 本项目**自己的状态文件**（每个项目一份，互不共享）。AI 每次启动必读（操作规则见中央 `PROMPT.md`）。
> 执行完成后，AI 必须更新「当前进度」和「最近变更」区块。
> ⚠️ 这是**状态**，不是规则；规则在只读的 `/home/toe/prompt/PROMPT.md`，不要改规则。

---

## 项目概述

ROS2 电机控制系统，面向竞技机器人平台（Robot_1A，**接球机**）。底层驱动层管理两类硬件：DJI CAN 总线电机（四舵轮底盘：四路 GM6020 转向 + 四路 GM3508 驱动）和 Unitree GO8010 原生串口电机（Delta 上抛臂三路 + 俯仰臂一路）。上层节点层提供 Delta 机械臂管理（软着陆 → 上抛 → 快速收拍 → 俯仰）和四舵轮底盘运动控制，对外暴露统一的 ROS2 话题接口。

## 代码根目录

> ⚠️ 代码写在**项目自己的代码目录**里。子 agent 执行前先 `cd` 到该目录。
> 本状态文件放在代码根目录。

- 本项目代码根目录：/home/toe/Robot_1A/USB2CAN_motor

---

## 文档索引

| 文档 | 说明 | 何时读 |
|------|------|--------|
| 中央 `PROMPT.md` | AI 运行手册（执行规则 + 工作模式，只读） | 每次必读 |
| 本目录 `design.md` | 需求 + 详细设计 + ADR | 开始新模块时 |
| 本目录 `USAGE.md` | 使用 / 构建 / 修改参数手册 | 要跑/构建/调参时 |
| 本目录 `tasks/progress.md` | 模块级总进度 checklist | 每次执行前确认进度 |
| 本目录 `tasks/<模块>.md` | 每模块最小子任务 | 实现该模块时 |
| 本文件 `CONTEXT.md` | 本项目当前状态 | 每次必读 |

---

## 当前进度

### 主流程
- [x] 需求确认
- [x] 底盘改四舵轮
- [x] 删除风车发球机（接球机不需要）
- [~] 俯仰从齿轮改杠杆（两连杆非线性，待函数拟合后实现坐标变换层）
- [ ] 上电标定（四舵轮 GM6020 零位偏移、轴距/轮距填参数）
- [ ] 集成测试

### 节点清单
| 节点 | 功能 | 状态 |
|------|------|------|
| `motor_control_node` | GO8010 串口 + DJI CAN 底层驱动 | ✅ |
| `omni_chassis_control_node`（实为四舵轮） | 四舵轮 FL/FR/RL/RR 运动控制 | ✅ 代码完成，待标定 |
| `delta_arm_manager_node` | Delta 臂软着陆→上抛→收拍→俯仰 | ✅ |

---

## 当前任务

**任务类型：** 硬件标定 + 联调准备  
**任务描述：** 上电前必须完成四舵轮标定：底盘处于正前方对齐时读取四路 GM6020 编码器角度(度) → 填入 `omni_chassis_params.yaml` 各 `*_steer_offset`；按实际机械尺寸填入 `wheel_base_x`/`wheel_base_y`/`wheel_radius`。俯仰杠杆非线性坐标变换待采集数据后用函数拟合实现（见「最近变更」杠杆计划）。  
**涉及模块：** omni_chassis_control_node、delta_arm_manager_node（tilt）  
**参考文档：** `omni_chassis_params.yaml`，`arm_config.yaml`  
**建议模型：** Sonnet

---

## 待确认问题

- 四舵轮实际轴距（`wheel_base_x`）和轮距（`wheel_base_y`）尚未填写，需现场测量
- 俯仰杠杆几何参数（电机连接点到支点距离、输出臂长）尚未量取

---

## 最近变更

- 2026-07-11: [现场联调·三处修复] ① 停车/空闲行为重写（`omni_chassis_control_node.cpp`）：原逻辑空闲时转向目标=当前反馈角，误差恒 0，既无保持力也不回零；改为**急停时锁存角度原地保持，非急停空闲时回机械零位**（编码器=steer_offset，上电解除急停后四轮自动对正，注意解除瞬间轮子会主动转到零位）。② 遥控轴映射改 REP-103（`rc_usb_control_node.cpp`）：旧映射 lx→linear.x/ly→linear.y 是全向轮开环语义，舵轮运动学 atan2(vy,vx) 按标准约定解释，导致前推杆被当成横移、转向打 90°（且 90° 在最短路径翻转临界点上，随机向左/向右）；现为 ly→linear.x(前+)、lx→-linear.y、rx→angular.z。③ 左侧驱动反装（`omni_chassis_params.yaml`）：实测前进命令右侧向前滚/左侧向后滚，左右轮组镜像安装，`fl_drive_direction`/`rl_drive_direction` 置 -1（与 offset±180 物理等效，但不动已标定零位）。PID 曾误判"力太小"上调过 GM6020 增益，已退回原值。待验证：前进/横移/自转三方向；自转时四轮呈 X 形为正常舵轮切线布局。

- 2026-07-11: [修复底盘不动·joint_name 不匹配] 根因：`omni_chassis_params.yaml` 里四轮 steer/drive 电机名是 `chassis_fl_steer`/`chassis_fl_drive` 等占位名，而 `motors.yaml` 实际电机 joint_name 是 `DJI6020_1..4`（转向）/`DJI3508_1..4`（驱动）；`motor_control_node.cpp:555` 按 joint_name 查 `motors_`，查不到就静默 `return`，命令全被丢，底盘完全不动（转向角优化里 `motor_states_` 同理查不到）。按现场映射（左前FL=3/右前FR=4/左后RL=2/右后RR=1）修正 yaml 四轮 steer/drive 电机名并 `colcon build` 装到 install 目录（注意：节点从 install/share 读配置，不读 start_chassis.sh 的 `-p config_file:=`，改源码 yaml 必须重新 build）。待现场核对转向/驱动 direction 与 steer_offset 标定。

- 2026-07-09: [整体迁移到 Robot_1A] 将所有开发从 `/home/toe/USB2CAN_motor`（错误目录）迁移到正确工作目录 `/home/toe/Robot_1A/USB2CAN_motor`。同步修复 `CLAUDE.md` 路径引用。

- 2026-07-09: [删除风车发球机 / 清理联调代码] 删除 `serve_windmill_manager.cpp/.hpp/serve_windmill_params.yaml`；从 `delta_arm_manager_node` 清除所有风车联调代码（WAIT_WIND_DONE 状态、maybeTriggerStrike、estimateLaunchHeight/FallTime、serve_trigger_pub_、wind_status_sub_ 及相关成员）；TILT_DOWN 改为命令角到位或 tilt_timeout_s 超时后直接进 RECOVER_READY；`serve_bringup.launch.py` 去掉风车节点；`arm_config.yaml` 删除 strike_delay/wind_done_timeout 等参数。`colcon build` 通过（0 error）。

- 2026-07-09: [四舵轮底盘] 将原全向轮（omni wheel）底盘改为四舵轮（swerve drive）矩形布局（FL/FR/RL/RR）。新建 `steer_wheel_kinematics.hpp/.cpp`（逆运动学 + 最短路径转向优化）；重写 `omni_chassis_control_node.cpp` 为四舵轮控制节点（GM6020 位置控制转向 + GM3508 速度控制驱动，estop 支持，停止时保持当前转向角）；更新 `omni_chassis_params.yaml`（`wheel_base_x`/`wheel_base_y`/`drive_gear_ratio` + FL/FR/RL/RR 各轮 steer/drive motor + offset/direction）；CMakeLists 加入 `steer_wheel_kinematics.cpp`，去掉 nav_msgs/rcl_interfaces 依赖。`colcon build` 通过（0 error）。**上电前必须标定**：正前方对齐时记录四路 GM6020 编码器角度 → 填入各 `*_steer_offset`。

- 2026-07-09: [俯仰改杠杆·计划] 俯仰传动从齿轮改为两连杆杠杆机构。计划分三阶段：① 现在：arm_config.yaml 参数已失效，需保守值上电探范围；② 采集：电机按角度步进，量取物理倾角，得 (θ_motor, θ_tilt) 映射对；③ 拟合 + 实现：在 `delta_arm_manager_node` 加 `tiltToMotor` / `motorToTilt` 坐标变换函数（查表或多项式），重力补偿前馈也做位置依赖。代码暂不改，待数据。

- 2026-07-09: [俯仰纳入 ArmTarget topic] `ArmTarget.msg` 新增 `float64 tilt_angle_rad` 字段；`delta_arm_manager_node` 新增成员 `tilt_target_cmd_rad_`，俯仰目标角**完全由 topic 决定**（0.0=不抬，保持躺平），删除 config `tilt.down_angle_rad` 及成员 `tilt_down_angle_rad_`。

- 2026-07-09: [击球逻辑重排：先俯仰后击球] 状态机重排为 READY → **TILT_AIM**（新增：三路 delta 锁零点，俯仰滑向 topic 击球角，命令到位+反馈进入 `tilt.position_tolerance` 容差才继续；`tilt_timeout_s` 超时兜底强制击球并 WARN）→ EXECUTE（上抛，俯仰保持击球角，原来是保持待机角）→ FAST_RETRACT（俯仰保持击球角）→ RECOVER_READY（俯仰回摆待机）→ READY。删除原 TILT_DOWN 状态（击球后才动俯仰的旧逻辑）。⚠️ 注意 `serve_test.tilt_timeout_s: 0.70` 现在是瞄准超时：3.6 rad ÷ rate 6 rad/s = 0.6 s 滑行 + 稳定时间，可能需调大。
