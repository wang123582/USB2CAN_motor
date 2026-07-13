# 需求与详细设计 (design.md)

> 本项目**自己的**需求 + 详细设计 + ADR（每项目一份）。
> 由需求确认和设计阶段的 AI 共同维护，人工审核后定稿。
>
> ⚠️ **章节互不删**：需求确认阶段只写/改「一、项目需求」；详细设计阶段只写/改「二、详细设计」。
> 任何阶段都**不得删除其它章节的内容**，只能追加或修改自己负责的章节。

---

## 一、项目需求

### 1.1 目标

为竞技机器人（发球机器人）提供完整的底层电机驱动和上层运动控制。核心场景：Delta 机械臂执行上抛 → 快速收拍 → 俯仰躺下 → 触发击球机构的发球动作序列，同时支持全向底盘移动。

### 1.2 功能边界

**包含：**
- DJI CAN 总线电机控制（GM3508，支持最多 4 路，全向底盘）
- Unitree GO8010 原生串口电机控制（4Mbps，支持多串口多 ID）
- Delta 机械臂管理：软着陆→上抛→快速收拍→俯仰→击球触发状态机
- 全向底盘运动控制（omni_chassis_control_node）
- RC USB 遥控输入（DJI USB 遥控 + 硬急停）
- 电机状态监控（motor_monitor_node）
- ROS2 消息：DJI/GO8010 命令/状态、ArmTarget、ControlFrequency

**不包含：**
- 视觉感知 / 球位置估计
- 自主路径规划
- 多机协同
- 下位机固件（只做 ROS2 驱动层）

### 1.3 接口约定

| 方向 | 话题 / 接口 | 消息类型 | 说明 |
|------|------------|---------|------|
| 输入 | `dji_motor_command` | `DJIMotorCommand` | DJI 电机扭矩命令 |
| 输入 | `dji_motor_command_advanced` | `DJIMotorCommandAdvanced` | DJI 高级命令（kp/kd/pos/vel/torque） |
| 输入 | `unitree_go8010_command` | `UnitreeGO8010Command` | GO8010 命令，优先按 `joint_name` 路由 |
| 输入 | `/delta_arm/target` | `ArmTarget` | Delta 臂目标角度（三路统一增量） |
| 输出 | `dji_motor_states` | `DJIMotorState[]` | DJI 电机反馈 |
| 输出 | `unitree_go8010_states` | `UnitreeGO8010State[]` | GO8010 反馈 |
| 输出 | `control_frequency` | `ControlFrequency` | 控制频率监控 |
| 输出 | `/delta_arm/ready` | `std_msgs/String` | 臂就绪通知 |
| 输出 | `/serve/trigger` | `std_msgs/Bool` | 击球触发信号 |
| 硬件 | `/dev/robocon_usb2can` | USB-CAN | DJI CAN 接口，波特率 921600 |
| 硬件 | `/dev/ttyUSB0` | RS485 | 俯仰 GO8010（id=0），4Mbps |
| 硬件 | `/dev/ttyUSB1/2/3` | RS485 | 上抛 GO8010（id=1/2/3），4Mbps |

### 1.4 验收标准

- [ ] 四路 DJI GM3508 上线后能接收扭矩命令并反馈状态（底盘能运动）
- [ ] 四路 GO8010 串口上线，`unitree_go8010_states` 持续有效反馈
- [ ] Delta 臂完整走一次发球序列（软着陆→上抛→快速收拍→俯仰→击球触发），无急停
- [ ] `arm_config.yaml` 中各行程限位生效：超限目标被拒绝，日志输出警告
- [ ] 换串口设备或改 ID 只需修改 `motors.yaml`，`arm_config.yaml` 不需动

---

## 二、详细设计

> 每个模块独立成节，模块间保持解耦，可独立测试。

### 模块：delta_arm_manager_node（现状）

**职责：** ROS2 Delta 机械臂管理节点，控制 3 路上抛 GO8010 和 1 路俯仰 GO8010，完成软着陆、上抛、快速收拍、俯仰躺下和击球触发。

**输入 / 输出：**
- 输入：`/delta_arm/target` (`ArmTarget`)、`unitree_go8010_states` (`UnitreeGO8010State`)
- 输出：`unitree_go8010_command` (`UnitreeGO8010Command`)、`/delta_arm/ready` (`std_msgs/String`)、`/serve/trigger` (`std_msgs/Bool`)

**核心数据结构：**
- `DeltaArmManager::State`：`INIT`、`SOFT_LANDING`、`READY`、`EXECUTE`、`FAST_RETRACT`、`TILT_DOWN`、`WAIT_STRIKE`、`TRIGGER_STRIKE`、`RECOVER_READY`
- 3 路电机状态数组：当前位置、速度、在线状态、反馈就绪、零点
- 俯仰电机状态：`arm_tilt_motor`，默认 `device=/dev/ttyUSB0`、`id=0`

**关键逻辑：**
- 启动后加载 `config/arm_config.yaml`，进入软着陆流程。
- 接收目标命令时要求三路 `target_angles` 一致，以统一相对增量执行。
- `EXECUTE` 上抛到 `ArmTarget.target_angles` 指定的统一相对角度。
- 上抛到位或超时后进入 `FAST_RETRACT`，三路上抛电机直接发布相对 0 度，快速收拍。
- 俯仰 GO8010 在 `FAST_RETRACT/TILT_DOWN` 阶段躺到配置角度，随后按测试延迟触发 `/serve/trigger`。
- 下臂 160 mm、上臂 230 mm 的弹道估算当前只用于日志和录像对比，不参与触发判断。

**测试策略：**
- 单元测试覆盖：当前未补单元测试；本次完成构建和 colcon 测试入口验证。
- Mock 对象：后续可 mock ROS2 publisher/subscription 和 GO8010 状态消息；硬件力矩/软着陆效果需实机验证。

---

### 模块：motor_control_node（GO8010 串口链路现状）

**职责：** 管理 DJI CAN 电机和 GO8010 原生串口电机；对上提供 ROS2 命令/状态 topic，对下完成 CAN/串口通信。

**输入 / 输出：**
- 输入：`dji_motor_command`、`dji_motor_command_advanced`、`unitree_go8010_command`
- 输出：`dji_motor_states`、`unitree_go8010_states`、`control_frequency`

**核心数据结构：**
- `UnitreeMotorNative`：GO8010 原生协议编解码，负责 17 字节命令和 16 字节反馈解析。
- `SerialInterface` / `SerialNetwork`：多串口管理，每个串口独立线程轮询。
- `motors.yaml serial_interfaces`：配置 GO8010 的 `device`、`id`、`direction`、`offset`、`gear_ratio`。

**关键逻辑：**
- `motor_control_node` 启动时读取 `motors.yaml`，初始化 CAN 接口和 GO8010 串口接口。
- `unitree_go8010_command` 优先按 `joint_name` 路由到对应 GO8010（`motors_` map 按名字查），fallback 到 `id+device` 精确匹配。
- 命令中的输出坐标先根据 `direction/offset` 转成电机原始坐标，再由 `UnitreeMotorNative` 打包为 GO8010 原生协议。
- 串口线程轮询每个 GO8010，解析反馈后发布 `unitree_go8010_states` 给 `delta_arm_manager_node`。

**测试策略：**
- 单元测试覆盖：当前未补单元测试；已完成构建和 colcon 测试入口验证。
- 硬件验证：需实机确认 `/dev/ttyUSB0/1/2/3` 设备映射、4Mbps 通信、四个 GO8010 的在线状态和方向/offset。

---

<!-- 按需复制模块节 -->

---

## 三、新功能记录

<!-- 有新功能时追加，不修改已有内容 -->

- 2026-07-03: delta 臂发球测试流程：上抛后快速收拍，俯仰 GO8010 躺下，按固定测试延迟触发击球机构；弹道时间估算只打印，不作为判断。

---

## 四、架构决策记录 (ADR)

<!-- 记录重要的设计决策，防止未来 AI 推翻已确认的方案 -->

| 日期 | 决策 | 原因 | 替代方案 |
|------|------|------|----------|
| 2026-07-03 | GO8010 使用原生串口协议（非 CAN）| GO8010 需 4Mbps 高速率，USB-CAN 适配器带宽不足；原生 RS485 延迟更低 | ROS2 hardware_interface + 统一 CAN 总线 |
| 2026-07-03 | Delta 臂三路增量必须统一（角度一致） | Delta 运动学要求三连杆同步；不等差会导致末端偏移或卡死 | 各路独立目标 + 正运动学求解 |
| 2026-07-04 | 按 `joint_name` 路由 GO8010 命令，`arm_config.yaml` 不再存 `device`/`id` | 避免两文件重复维护，减少换串口时遗漏改一处的配置漂移风险 | 保留双文件、运行时校验一致性 |
| 2026-07-04 | 每路独立梯形规划器，`target_deltas_rad_[3]` 替代标量 | 实测三路电机行程不等（最大差 0.045 rad），需各路独立上限校验和规划 | 统一标量 + 取三路最小值限速 |
