# 总体进度 (tasks/progress.md)

> 模块级进度总览，由**主 Agent**维护。
> 每个模块对应一个 `tasks/<模块>.md` 子任务清单。
> 状态：`[ ]` 未开始 / `[~]` 进行中 / `[x]` 完成 / `⚠️ blocked` 受阻待裁决。

---

## 模块进度

<!-- 任务划分阶段在此列出所有模块，格式：- [ ] 模块名 → tasks/模块名.md -->

- [x] delta_arm_manager_node 导入 → tasks/delta_arm_manager_node.md
- [x] config_fix：修复构造函数默认值 / catch 静默吞异常 / motor_ids_ 冲突 → tasks/config_fix.md
- [x] name_routing：按 joint_name 路由 GO8010 命令，arm_config.yaml 删除冗余 device/id → tasks/config_fix.md
- [x] retract_speedup：收拍回落提速（取消钳位 + 反向梯形规划速度前馈 + 激进 retract 参数）
- [x] tilt_in_topic：俯仰目标角纳入 ArmTarget topic（新增 tilt_angle_rad 字段，完全由 topic 决定，删除 config down_angle_rad）
- [x] tilt_aim_first：击球逻辑重排，先俯仰瞄准（TILT_AIM，反馈到位确认）再上抛击球，删除旧 TILT_DOWN 状态

---

## Blocked（受阻，等用户裁决）

<!-- 命中停点规则时，主 Agent 把模块移到这里，并在 CONTEXT.md「待确认问题」写清原因 -->

- delta_arm_manager_node 硬件联调参数调整 ⚠️ blocked：缺少实机/录像反馈，无法判断 `tilt.down_angle_rad`、`serve_test.strike_delay_override_s`、`retract.*` 和 `tilt.*` 的调整方向；详见 `CONTEXT.md`「待确认问题」。

---

## 待提交 Git（等用户说「可以」）

<!-- 模块测试通过后列在这里，提醒用户批准提交；夜间模式也在此累积 -->

- delta_arm_manager_node 导入已就绪；验证：`colcon build --packages-select motor_control_ros2` 通过，`colcon test --packages-select motor_control_ros2` 通过（0 tests）。

---

## 并行批次记录

<!-- 主 Agent 记录每批并行派生了哪些子 agent、模块是否真解耦、共享文件是否需先串行 -->

_（暂无）_
