# 项目上下文 (CONTEXT.md)
> 本项目**自己的状态文件**（每个项目一份，互不共享）。AI 每次启动必读（操作规则见中央 `PROMPT.md`）。
> 执行完成后，AI 必须更新「当前进度」和「最近变更」区块。
> ⚠️ 这是**状态**，不是规则；规则在只读的 `/home/toe/prompt/PROMPT.md`，不要改规则。

---

## 项目概述

ROS2 电机控制系统，面向竞技机器人平台。底层驱动层管理两类硬件：DJI CAN 总线电机（全向底盘四路 GM3508）和 Unitree GO8010 原生串口电机（Delta 上抛臂三路 + 俯仰臂一路）。上层节点层提供 Delta 机械臂管理（软着陆 → 上抛 → 快速收拍 → 俯仰触发发球）和全向底盘运动控制，对外暴露统一的 ROS2 话题接口。

## 代码根目录

> ⚠️ 代码写在**项目自己的代码目录**里。子 agent 执行前先 `cd` 到该目录。
> 本状态文件放在代码根目录。

- 本项目代码根目录：/home/toe/USB2CAN_motor

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
- [x] 详细设计
- [x] 任务划分
- [~] 模块实现（见 tasks/progress.md）
- [ ] 集成测试（待上电硬件联调）

### 新功能
- [x] feature: GO8010 原生串口链路（motor_control_node）
- [x] feature: Delta 机械臂管理节点（delta_arm_manager_node）
- [x] feature: 按 joint_name 路由 GO8010 命令（名字路由重构）
- [x] feature: 实测行程约束（每路独立 max_delta_rad + 梯形规划）

### Bug 修复
- [x] bugfix: delta_arm_manager_node 构造函数默认值与 arm_config.yaml 严重不符
- [x] bugfix: catch 块静默吞异常导致错误参数无声运行
- [x] bugfix: motor_ids_ 默认 {0,0,0} 与 tilt id=0 冲突

---

## 当前任务

**任务类型：** 硬件联调 / 参数调整
**任务描述：** 名字路由重构完成，构建通过。换串口或改 ID 只需改 `motors.yaml`，`arm_config.yaml` 的 `motors`/`tilt` 段已删除 `device`/`id`（仅保留 `name` 和 `max_delta_rad`）。下一步上电验证各路通信，再进行动作联调。
**涉及模块：** motor_control_node，delta_arm_manager_node
**参考文档：** USAGE.md，tasks/progress.md
**建议模型：** Sonnet（硬件联调）

---

## 待确认问题 

_（当前无待确认问题）_

---

## 最近变更

- 2026-07-04: [收拍回落提速] 用户反馈 `FAST_RETRACT` 收拍回落太慢，速度优先。三项改动：①`publishCommand` 加 `bypass_clamp` 形参，收拍时跳过 `max_position_error` 钳位，让 PD 看到完整误差、跑满回落力矩；②`FAST_RETRACT` 从「直接甩零点位置 + vel=0 硬拉」改为反向梯形规划：`enterFastRetract` 让 `planned_deltas_rad_` 从当前顶点出发，规划器驱动回 0 并带 `velocity_target` 前馈，仅贴近零点时收速度防砸零点；用独立高速上限 `retract_max_velocity_`/`retract_max_acceleration_`；③`arm_config.yaml` retract 段调激进：`kp` 0.80→1.50、`torque_ff` -0.3→-0.5，新增 `max_velocity=15`、`max_acceleration=80`（均高于 execute 段）。验证：`colcon build --packages-select motor_control_ros2` 通过（0 warning，0 error）。待上电验证回落速度与零点是否回弹。
- 2026-07-04: [实测行程约束] 根据硬件实测数据（Motor1=92.358°/1.612rad、Motor2=92.290°/1.611rad、Motor3=89.748°/1.567rad）写入 `arm_config.yaml` `motors[i].max_delta_rad`；`safety.angle_diff_tolerance=0.0873`（5°）放宽等差检查；`delta_arm_manager_node` 三路规划变量改为数组，EXECUTE 状态每路独立梯形规划；`armTargetCallback` 逐路校验上限并拒绝超限命令。验证：`colcon build --packages-select motor_control_ros2` 通过（0 warning，0 error）。
- 2026-07-04: [名字路由重构] 实现按 `joint_name` 路由命令，消除 `arm_config.yaml` 与 `motors.yaml` 重复维护 `device`/`id`：①`UnitreeGO8010Command.msg` 加 `string joint_name` 字段；②`motor_control_node` `unitreeGoCommandCallback` 优先按名字查 `motors_` map 路由，fallback 到 `id+device`；③`delta_arm_manager_node` `publishCommand`/`publishTiltCommand` 改填 `cmd.joint_name`；④`arm_config.yaml` `motors`/`tilt` 段删除 `device`/`id`（仅保留 `name`、`max_delta_rad`）；同步修复 `.cpp` 用 `target_deltas_rad_[i]`/`planned_deltas_rad_[i]`/`current_planned_vels_[i]` 数组对齐 header。验证：`colcon build --packages-select motor_control_ros2` 通过（1 warning，无错误）。
- 2026-07-04: [config_fix] 修复 `delta_arm_manager_node.cpp` 三处高危问题：①catch 改为 ERROR + rethrow；②8 个构造函数默认值对齐 `arm_config.yaml`（`max_velocity_` 0.8→10、`tracking_error_pause_` 0.2→3.0、`gravity_compensation_torque_` 0→0.8 等）；③`motor_ids_` `{0,0,0}`→`{1,2,3}`。验证：`colcon build --packages-select motor_control_ros2` 通过。
- 2026-07-04: [配置审查] 检查 `motors.yaml` 与 `arm_config.yaml` 一致性：发现两个高危问题——①`delta_arm_manager_node.cpp` 构造函数默认值与 `arm_config.yaml` 严重不符（`max_velocity_` 差 12.5x、`tracking_error_pause_` 差 15x、`gravity_compensation_torque_` 为 0 vs 0.8），catch 块静默吞掉异常导致错误参数无声运行；②`motor_ids_` 默认 `{0,0,0}` 与 tilt 电机 `id=0` 冲突。已建 `tasks/config_fix.md` 记录修复子任务；未改代码，等待用户确认后执行。
- 2026-07-03: [motor_control_node] 融入 GO8010 原生串口链路：新增 `SerialInterface`、`UnitreeMotorNative`，扩展 `ConfigParser` 支持 `serial_interfaces`，`motor_control_node` 订阅 `unitree_go8010_command`、发布 `unitree_go8010_states`，并按 `device + id` 路由多串口同 ID GO8010；`motors.yaml` 增加 `/dev/ttyUSB0/1/2/3` 四路 GO8010 配置。验证：`colcon build --packages-select motor_control_ros2` 通过，`colcon test --packages-select motor_control_ros2` 通过（0 tests）。
- 2026-07-03: [工作流执行] 按 `.CLAUDE.md`/`CLAUDE.md` 加载中央规则、本项目 `CONTEXT.md`、`design.md`、`tasks/progress.md` 和 `PROMPT.md`；确认当前任务为硬件联调参数调整，但缺少实机/录像反馈，已在「待确认问题」记录所需输入，未改代码或参数。
- 2026-07-03: [delta_arm_manager_node] 修改发球测试状态机：`EXECUTE` 上抛后进入 `FAST_RETRACT` 直接回相对 0，俯仰 GO8010（默认 `/dev/ttyUSB0`、`id=0`）躺到测试角 `1.0 rad`，按 `serve_test.strike_delay_override_s` 触发 `/serve/trigger`；下臂 160 mm、上臂 230 mm 的弹道估算只用于日志/录像对比。验证：`colcon build --packages-select motor_control_ros2` 通过，`colcon test --packages-select motor_control_ros2` 通过（0 tests）。
- 2026-07-03: [delta_arm_manager_node] 从 `/home/toe/111/USB2CAN_motor` 导入 Delta 机械臂管理节点、直接依赖头文件、3 个消息文件和 `arm_config.yaml`；更新 `motor_control_ros2` CMake 构建/安装目标。验证：`colcon build --packages-select motor_control_ros2` 通过，`colcon test --packages-select motor_control_ros2` 通过（0 tests）；仍需硬件联调。
- 2026-07-03: [需求确认] 用户要求“执行下一步”，已读取 `CONTEXT.md`、`design.md`、`USAGE.md`、`tasks/progress.md`；当前任务缺少明确开发目标，按规则停止并等待用户确认。
- 2026-07-03: [工作流接入] 创建本项目状态骨架，确认代码根目录为 `/home/toe/USB2CAN_motor`；未预填项目需求、模块设计或任务内容。
