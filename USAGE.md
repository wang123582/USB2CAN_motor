# 使用 / 构建 / 修改手册 (USAGE.md)

> 记录每个项目「怎么跑、怎么构建、常改的参数在哪」。
> ⚠️ **改动了影响运行 / 构建 / 参数的代码后，必须同步更新本文件对应小节。**
> **按需填写**：只为你已经接触过的项目/模块填，不必一次写全（模块解耦，边做边长）。

---

## 项目：/home/toe/USB2CAN_motor

### 使用方法（怎么跑）
- 启动命令：先启动 `ros2 run motor_control_ros2 motor_control_node`，再启动 `ros2 run motor_control_ros2 delta_arm_manager_node`
- 依赖的硬件 / ROS2 话题 / 环境：依赖 ROS2 环境、`motor_control_ros2` 已构建并 source；GO8010 串口链路由 `motor_control_node` 订阅 `unitree_go8010_command` 并发布 `unitree_go8010_states`。
- 常见用法示例：确认 `/dev/robocon_usb2can` 和 `/dev/ttyUSB0/1/2/3` 存在后，启动 `motor_control_node`，看到 GO8010 状态在线后再启动 `delta_arm_manager_node`。

### 构建方法（怎么装 / 编译）
- 环境与依赖安装：ROS2/colcon 工作区环境。
- 构建 / 编译命令：`colcon build --packages-select motor_control_ros2`
- 配置文件位置：`src/motor_control_ros2/config/motors.yaml`（硬件接口/电机列表），`src/motor_control_ros2/config/arm_config.yaml`（臂动作参数）

### 修改方法（常改参数）
> 把"经常需要调的参数"列在这，免得每次翻代码找。

| 参数 | 文件位置 | 含义 | 默认值 / 建议范围 |
|------|---------|------|------------------|
| `control_frequency` | `src/motor_control_ros2/config/arm_config.yaml` | Delta 机械臂控制频率 | 按硬件能力调整 |
| `initialization.*` | `src/motor_control_ros2/config/arm_config.yaml` | 软着陆力矩、超时、稳定阈值、阻尼、重力补偿 | 需硬件联调 |
| `pd.*` | `src/motor_control_ros2/config/arm_config.yaml` | 位置控制刚度/阻尼 | 需硬件联调 |
| `motion_profile.*` | `src/motor_control_ros2/config/arm_config.yaml` | 梯形规划速度、加速度、减速比例增益 | 需硬件联调 |
| `serve_test.strike_delay_override_s` | `src/motor_control_ros2/config/arm_config.yaml` | 上抛后触发击球的测试固定延迟；`>=0` 时优先于模型时间 | 先用 `0.35`，录像后调 |
| `serve_test.arm_lower_length_m` / `arm_upper_length_m` | `src/motor_control_ros2/config/arm_config.yaml` | 弹道测试估算用机械臂长度 | `0.160` / `0.230` |
| `retract.*` | `src/motor_control_ros2/config/arm_config.yaml` | 快速收拍回相对 0 的 GO8010 增益/前馈 | 需硬件联调 |
| `tilt.*` | `src/motor_control_ros2/config/arm_config.yaml` | 俯仰 GO8010 设备、躺下角和增益 | 默认 `/dev/ttyUSB0`、`id=0`、`down_angle_rad=1.0` |
| `serial_interfaces` | `src/motor_control_ros2/config/motors.yaml` | GO8010 原生串口电机配置，`device + id` 要与 `arm_config.yaml` 匹配 | 默认 `/dev/ttyUSB0/1/2/3`，各 `id=0` |
| `direction` / `offset` / `gear_ratio` | `src/motor_control_ros2/config/motors.yaml` | GO8010 反馈和命令的方向、零位、减速比换算 | 需实机标定 |

---

<!-- 每多接触一个代码根目录 / 模块，复制上面一节 -->
