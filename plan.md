# 测试计划：delta_arm_manager_node 硬件联调

## Context

代码已导入并通过编译（`colcon build` + `colcon test` 均通过）。当前阶段是首次硬件联调：
- 4 路 GO8010 串口电机（`/dev/ttyUSB0/1/2/3`）已配置
- `motor_control_node` 负责串口通信，发布 `unitree_go8010_states`，订阅 `unitree_go8010_command`
- `delta_arm_manager_node` 负责状态机控制，订阅状态反馈，发布命令

目标：验证端到端链路通，状态机能走完一次完整的发球动作（INIT → READY → EXECUTE → FAST_RETRACT → TILT_DOWN → WAIT_STRIKE → TRIGGER_STRIKE → RECOVER_READY）。

---

## 步骤 1：构建 & 预检

```bash
cd /home/toe/USB2CAN_motor
colcon build --packages-select motor_control_ros2
source install/setup.bash

# 确认设备节点存在
ls /dev/ttyUSB*
# 预期：/dev/ttyUSB0  /dev/ttyUSB1  /dev/ttyUSB2  /dev/ttyUSB3
```

---

## 步骤 2：启动 motor_control_node（终端 A）

```bash
cd /home/toe/USB2CAN_motor && source install/setup.bash
ros2 run motor_control_ros2 motor_control_node
```

**观察点（30 s 内）：**
- 日志无 `Failed to open` 报错 → 4 路串口全部打开
- `unitree_go8010_states` 开始发布，`online: true` 覆盖 4 个 `joint_name`：
  - `arm_delta_motor_1`（/dev/ttyUSB1, id=1）
  - `arm_delta_motor_2`（/dev/ttyUSB2, id=2）
  - `arm_delta_motor_3`（/dev/ttyUSB3, id=3）
  - `arm_tilt_motor`（/dev/ttyUSB0, id=0）

```bash
# 终端 B 监控电机状态
ros2 topic echo /unitree_go8010_states
```

---

## 步骤 3：启动 delta_arm_manager_node（终端 C）

```bash
cd /home/toe/USB2CAN_motor && source install/setup.bash
ros2 run motor_control_ros2 delta_arm_manager_node
```

**观察点（软着陆阶段，最长 5 s）：**
1. 日志打印 `配置加载完成` → YAML 读取正常
2. 日志打印 `INIT → SOFT_LANDING`：3 路上抛电机以 `downward_torque=-0.15 Nm + landing_kd=0.05` 缓慢下压
3. 三电机速度均 < 0.3 rad/s 且稳定 0.8 s 后，日志打印 `SOFT_LANDING → READY`
4. `/delta_arm/ready` 发布 `"READY"` 字符串

```bash
# 终端 D 监控 ready 信号
ros2 topic echo /delta_arm/ready
```

若 5 s 超时强制进入 READY，说明软着陆力矩/阻尼需调整（见调参建议）。

---

## 步骤 4：发送测试命令（触发一次完整动作）

确认 `/delta_arm/ready` 已发布后：

```bash
ros2 topic pub --once /delta_arm/target motor_control_ros2/msg/ArmTarget \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
    target_angles: [0.3, 0.3, 0.3],
    execute: true}'
```

**预期状态机流转（观察日志）：**

| 状态 | 触发条件 | 预期耗时 |
|------|---------|---------|
| READY → EXECUTE | 收到命令 | 即时 |
| EXECUTE → FAST_RETRACT | 三电机到达目标 ±0.05 rad | 取决于 kp/kd |
| FAST_RETRACT → TILT_DOWN | 三电机回零 或超时 0.30 s | ~0.3 s |
| TILT_DOWN → WAIT_STRIKE | 俯仰电机到位 或超时 0.50 s | ~0.5 s |
| WAIT_STRIKE → TRIGGER_STRIKE | 从上抛完成计时 0.35 s 到达 | ~0.35 s |
| TRIGGER_STRIKE → RECOVER_READY | 触发脉冲 0.05 s 后 | ~0.05 s |
| RECOVER_READY → READY | 立即 | 即时 |

```bash
# 监控触发信号
ros2 topic echo /serve/trigger
```

---

## 步骤 5：验收标准

- [ ] 4 路 GO8010 全部 `online: true`
- [ ] delta_arm_manager 正常完成软着陆（READY 在 5 s 内出现，非超时强制）
- [ ] 日志顺序打印完整状态机流转
- [ ] `/serve/trigger` 收到 `data: true`
- [ ] 动作完成后回到 READY，可重复发命令

---

## 调参建议（若联调发现问题）

| 现象 | 调整参数（arm_config.yaml） |
|------|--------------------------|
| 软着陆超时，速度降不下来 | 增大 `initialization.downward_torque`（负值更负），增大 `initialization.landing_kd` |
| EXECUTE 阶段电机抖振 | 降低 `pd.kp` 或 `motion_profile.planner_p_gain` |
| FAST_RETRACT 超时未到位 | 增大 `retract.kp` |
| 俯仰电机到位慢 | 调整 `tilt.kp` 和 `tilt.down_angle_rad` |
| 击球时序偏早/偏晚 | 调整 `serve_test.strike_delay_override_s`（录像测量实际下落时间） |
| 串口无反馈 / online=false | 检查 `motors.yaml` 中 `device` 和 `baudrate`（当前 4000000） |

---

## 关键文件

- `src/motor_control_ros2/config/motors.yaml` — 串口设备 + 电机 ID 配置
- `src/motor_control_ros2/config/arm_config.yaml` — 臂控制所有参数
- `src/motor_control_ros2/src/nodes/delta_arm_manager_node.cpp` — 状态机实现
- `src/motor_control_ros2/src/nodes/motor_control_node.cpp` — 串口通信 + 状态发布
