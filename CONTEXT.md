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

**任务类型：** 硬件联调（delta+windmill 全链路）
**任务描述：** 上抛提速、俯仰简化、windmill 整合接线均已改完、`colcon build` 通过，进入全链路联调。启动顺序：motor_control_node → serve_windmill_manager（`/serve/command "start"` 引拍到 WIND_UP 待命）→ delta_arm_manager 跑发球。验证链路：delta 到点发 `/serve/trigger` → wind FIRE 击球 → FREE_WHEEL→CATCH→GRAVITY_HOMING→WIND_UP → delta 见"接住+重力归零完成"日志（wind 重回 WIND_UP）才俯仰回摆（非走 `wind_done_timeout_s`=8.0 超时兜底）。同时仍要看：上抛出手速度/球高、满速撞挡块冲击（挡块/减速箱、撞停对齐日志、反弹）、俯仰待机位是否被恒定 ff 顶响。调参入口见 USAGE.md / arm_config.yaml 各段注释。
**涉及模块：** delta_arm_manager_node（+ 后续 serve_windmill_manager 状态订阅）
**参考文档：** USAGE.md，CONTEXT.md 最近变更
**建议模型：** Sonnet（硬件联调）

---

## 待确认问题 

_（当前无待确认问题）_

---

## 最近变更

- 2026-07-06: [俯仰回摆时机·等重力归零完成] 用户要求：俯仰电机归位（回摆）应在 windmill 打完之后、**重力归零完成**才发生，而非现在的"CATCH 一结束就回摆"。查 windmill 状态机（serve_windmill_manager.cpp）击球后循环：FIRE→FREE_WHEEL→CATCH→GRAVITY_HOMING→WIND_UP（`handleGravityHoming` 归零稳定 `homing_stable_duration_s` 后才 `transitionTo(WIND_UP)`；`handleCatch` 接住完 → GRAVITY_HOMING）。原 WAIT_WIND_DONE 判据 `wind_catch_seen_ && wind_status_ != "CATCH"`——离开 CATCH 只是进 GRAVITY_HOMING（归零**刚开始**）就回摆了。改判据为 `wind_catch_seen_ && wind_status_ == "WIND_UP"`：见过 CATCH 后等 windmill 重回 WIND_UP（= 重力归零已完成，唯一入口是归零完），俯仰才回摆。同步改日志（"接住+重力归零完成"）、hpp State 枚举注释、类头状态机注释。`wind_done_timeout_s`=8.0 超时兜底不变（防 windmill 掉线卡死）。验证：`colcon build` 通过。**待上电确认**：俯仰是否等到 windmill 归零完（重回引拍 WIND_UP）才回摆、8s 兜底是否够（归零慢则加大 wind_done_timeout_s）。

- 2026-07-05: [挥拍解耦提前] 联调日志：收拍 0.205s→抬俯仰 0.390s→进 WAIT_STRIKE 时已 launch 后 0.595s，而 strike_delay=0.35 早被超过→**挥拍实际卡在 0.595s、strike_delay 失效**（被"收拍+抬俯仰"总时长卡死）。用户要"挥拍早一点"。改：**把 windmill 开火与状态机进度解耦**——新增 `maybeTriggerStrike(force=false)`，在 FAST_RETRACT/TILT_DOWN 里每周期检查，launch 后一过 strike_delay 立刻发一次开火（不等收拍/抬俯仰完）。删掉 WAIT_STRIKE/TRIGGER_STRIKE 两个状态：TILT_DOWN 里抬俯仰 + maybeTriggerStrike，一旦已发即进 WAIT_WIND_DONE（抬俯仰 timeout 则 force 强发兜底）。`strike_delay_override_s` 0.35→0.25（现直接生效=挥拍时机，windmill 开火→撞球约 +0.267s，故实际击球≈值+0.267s）。State 枚举/头注释同步删两状态。验证：`colcon build` 通过。**待上电确认**：挥拍是否提前到 launch 后 ~0.25s（看"已触发/serve/trigger（挥拍），launch 后 x.xxx s"日志）、击球点是否对（早了加大 strike_delay，晚了减小）。⚠️ 挥拍现可能在俯仰抬到位前发生——若俯仰与 windmill 共用空间需防撞，独立则无碍。

- 2026-07-05: [一键 launch] 新增 `launch/serve_bringup.launch.py`，**只拉起三节点**（motor_control_node + serve_windmill_manager + delta_arm_manager），无自动发命令、无 timer——用户要求引拍 start / 发球 target 都自己手动发（命令写在 launch 文件头注释当备忘）。CMakeLists 加 `install(DIRECTORY launch)`。用法 `ros2 launch motor_control_ros2 serve_bringup.launch.py`。验证：`colcon build` 通过、launch 已装、`py_compile` 语法 OK。

- 2026-07-05: [windmill 联调·接线] 按已定方案给 delta 加 windmill 整合（wind 手动先引拍就位、delta 只发一次开火触发、时机=delta 算的 strike_delay、俯仰等 wind 整循环 CATCH 完毕再回摆、不传数字）。改动全在 delta_arm_manager：①订阅 windmill `/serve/status`(String) → `windStatusCallback` 缓存 `wind_status_`；②State 枚举加 `WAIT_WIND_DONE`；③`TRIGGER_STRIKE` 发完触发脉冲改为进 `WAIT_WIND_DONE`（原直接 RECOVER_READY），并复位 `wind_catch_seen_`；④`WAIT_WIND_DONE`：三路锁零点、俯仰保持击球位，检测"见过 CATCH 又离开 CATCH=接住完毕"→ RECOVER_READY 回摆；带 `wind_done_timeout_s`(8.0) 超时兜底防 windmill 掉线卡死。config serve_test 加 `wind_done_timeout_s`。windmill 侧未改（自身 CATCH→GRAVITY_HOMING→自动 WIND_UP 重新引拍）。话题：delta→wind `/serve/trigger`(Bool)，wind→delta `/serve/status`(String，值 IDLE/GRAVITY_HOMING/WIND_UP/FIRE/FREE_WHEEL/CATCH/E_STOP)。验证：`colcon build` 通过。**联调启动顺序**：先起 motor_control_node → 起 serve_windmill_manager 并手动 `/serve/command "start"` 让它引拍到 WIND_UP 待命 → 起 delta_arm_manager 跑发球；delta 到点发 trigger→wind FIRE→…→CATCH→homing，delta 见 CATCH 完毕才回摆。无 launch 文件，各节点手动 `ros2 run`。

- 2026-07-05: [上抛提速·去顶部软压] 用户反馈上抛太慢，讨论定位：EXECUTE 上抛用 motion_profile(max_velocity=10/max_acceleration=50)，~1.6rad 行程三角峰速≈√(50×1.6)≈8.9 已加速度受限；更关键——用户确认**球在顶部靠机械臂撞挡块急停才脱手**(出手速度=撞挡块前臂速)，而之前的"顶部软压"(top_approach_velocity=3.5 在最后 0.2rad 降速)正好把出手速度砍到 3.5=头号元凶；且用户确认**电机有力气**(非扭矩饱和)。改：①EXECUTE 删掉 vcap 软压逻辑——上抛全程满加速冲挡块、不减速(press_margin 让规划不进减速段)；②`max_acceleration` 50→150(√(2×150×1.6)≈21.9rad/s)、`max_velocity` 10→22；③删 `top_approach_velocity` 成员/loader/config(撞停脱手机制下"缓一下"是反的)，保留 `top_approach_band_rad` 供撞停判据。cpp 默认同步。验证：`colcon build` 通过。**⚠️ 风险：满速(~22rad/s)撞挡块，挡块/减速箱冲击寿命**——上电从 accel=150 起，不够高再加、太猛降一点；关注"三路撞停对齐"日志是否正常(非 timeout)、撞停后是否剧烈反弹影响判据。仍要更高→继续抬 max_acceleration。

- 2026-07-05: [俯仰简化] 用户："俯仰太复杂，应一直有一个向上克服臂重的力，其余从简"，先做简化版测试（windmill 整合下一步）。洞察：原来抬起/回摆拆两套(raise 梯形+ff托住、return 斜坡+ff=0靠重力带回)是因**没有恒定重力补偿**、两方向不对称。改成**一直挂恒定向上前馈 `tilt_hold_ff`(=1.1) 抵消臂重→两方向对称→单一 PD 一套搞定**。删 `tiltCommandRaise`/`tiltCommandReturn`→合成 `tiltCommand(target_rel,dt)`：命令角按单一 `tilt_rate_rad_s`(6.0) 限速滑向目标 + 恒定 hold_ff + 单一 tilt_kp/kd(2.0/0.30)。删成员 tilt_torque_ff_/raise_rate/raise_accel/raise_p_gain/return_kp/return_kd/return_rate/ready_torque_ff/tilt_cmd_vel_；保留零点捕获、max_position_error 钳位。7 处调用点改：READY/EXECUTE→`tiltCommand(ready)`、FAST_RETRACT/TILT_DOWN/WAIT_STRIKE/TRIGGER_STRIKE→`tiltCommand(down)`、RECOVER_READY→`tiltCommand(ready)`。config tilt 段：kp/kd/hold_torque_ff/rate_rad_s/position_tolerance/max_position_error，删 raise/return/torque_ff/ready_torque_ff。验证：`colcon build` 通过。**待上电测**：①抬起到位保持、回摆躺回是否都顺；②**待机位(0)是否被恒定向上 ff 顶离挡块/嗡嗡响**——若响则待机需撤 ff（当时问过用户"躺平是否靠挡块休息"未定，先按"一直托"实现）。调参：臂往 0 掉→加大 hold_torque_ff；待机顶挡块→调小。**未做**：windmill 整合（时机=delta 算的击打延迟触发 fire、俯仰等 wind 整循环 CATCH 完毕再回摆、wind 手动先引拍就位）——本轮只简化俯仰。

- 2026-07-05: [收拍提速] 顶部限位方案上电见效：读 csv 确认 delta 同步已好——三路间 av 均差降到 0.26-0.37(原 3.99)、符号割裂 发1/发3=0 行、发2 仅 3 行且全在 ad=-0.04 的**底部过零反弹**(中段全程 0 割裂)、下降剖面干净无锯齿。用户反馈"除速度不够外 delta 没问题了"。定位：下降峰速三发都压平在 -12=撞 `retract.max_velocity=12` 天花板；且 ~1.7rad 短行程三角轨迹峰速≈√(80×1.7)≈11.7 → 速度**同时受 max_velocity 和 max_acceleration 卡住**。改：`retract.max_velocity` 12→18、`max_acceleration` 80→130(√(130×1.7)≈14.9)、`bottom_soft_rad` 0.25→0.35(提速后到底更猛，提前撤向下前馈防反弹)。cpp 默认值同步。验证：`colcon build` 通过。**风险：底部无限位挡块，越快过零反弹越明显**——待上电看发2 那种底部 +/- 反弹是否加剧；若加剧→底部也加挡块 或 收 torque_ff/再加大 bottom_soft。仍不够快→继续抬 max_acceleration(主瓶颈)，或收 torque_ff(-0.5)给更多向下力让实速跟上更陡的规划。

- 2026-07-05: [换思路·顶部限位硬件同步] 用户否定"软件磨顺滑"路线：读新 csv 印证 lead_cap 0.10→0.30 只是把 terr 钉的位置从 -0.10 挪到 -0.30，单路速度摆幅仍 13-14rad/s、发3 三路间差冲到 3.99；且滞后补偿没救回来。用户拍板换硬件方案（AskUserQuestion 确认：**限位装顶部**、倾倒用**现有俯仰电机**）：满速上抛把球抛高→撞顶部机械挡块→三路被挡块顶到同位置、速度**一起归零**（硬件同步，解决 66Hz 反馈下软件压不平的锯齿）→"这个速度可以缓一下"(接近顶降速软压)→再快速向下→俯仰倾倒发球。核心要求仍是**不卡顿、尤其不要"有的正有的已负"**。改动：①EXECUTE 重写——目标压到 `限位+top_press_margin`(0.10) 持续顶向挡块，实际角进入 `target-top_approach_band`(0.20) 后速度上限降到 `top_approach_velocity`(3.5) 软压；转收拍判据从 `allMotorsReached`(只看位置、到达时间各异致割裂)改为**三路都到顶带内且实际速度都 <`stop_settle_vel`(0.5)** 的撞停对齐判据。②FAST_RETRACT 简化——顶部已对齐，改共享**开环**梯形回零(距零 max_v/p_gain 内减速)+底部软着陆(`bottom_soft_rad`=0.25)，**删掉** lead_cap 牵引/max_actual 闭环减速/滞后预测(连同上一版加的 `last_feedback_time_` 成员与 `retract_lead_cap_`/`lead_cap_rad` 一并移除)。config：motion_profile 加 4 个 top_* 旋钮；retract 段删 lead_cap_rad。`allMotorsReached()` 定义保留但不再调用。验证：`colcon build` 通过。**前提：硬件需先装顶部限位挡块**，否则撞停判据永远靠 top_idle_timeout(1.0s)超时兜底。待上电抓 RETRACT_CSV 看切换瞬间三路是否一起归零、中段是否无锯齿无割裂。

- 2026-07-05: [收拍同步·滞后补偿+解耦限速+软着陆]（已被上条"顶部限位"方案取代：滞后预测/lead_cap 均移除） 用户已把 `lead_cap` 0.10→0.30 试成功，并发现**降击打高度能缓解不同步**——印证速度是根因（高度低→收拍起点低→下降速度低→反馈滞后影响小）。读 6 发 retract.csv 定位：①同步版**已把中段的 +/- 符号割裂治好**（中段 ad>0.15 无一行符号割裂，三路同符号、位置差<0.01）；②残留卡顿是两件事——(a) 中段**共模锯齿**：单路速度时间摆幅约 12rad/s（-0.7↔-3.5 反复），terr 全程被钉在 -0.10=-lead_cap；根因 `lead_cap=0.10 < 反馈滞后 0.22`（控制 200Hz 但反馈仅 ~66Hz，CSV 里 av 每 3 拍才更新一次）→规划每 15ms 撞 lead_floor 被强制 vel 归零→弛张振荡；(b) 底部 ad≈0 仅 1 次触底 +/- 反弹(`av=(0.23,-1.97,0.65)`)。用户选 1+2+3 全改：①**反馈滞后速度外推补偿**——`motorStateCallback` 记 `last_feedback_time_[i]`(hpp 新增成员)，FAST_RETRACT 里 `max_actual` 改用 `ad+vel*age`(age 上限 0.03s 防掉线发散)，牵引绳比"电机现在真实在哪"→根除滞后锯齿；②`retract.max_velocity` 15→12 解耦下降速度与高度(高度由 EXECUTE 决定)；③`bottom_soft_rad` 0.15→0.25 让三路更早收速防砸床反弹。cpp 默认值同步(15→12/0.10→0.30/0.15→0.25)。验证：`colcon build` 通过。**待上电抓 RETRACT_CSV**：中段单路锯齿摆幅是否显著变小、terr 是否脱离 -lead_cap、底部是否不再符号割裂。仍抖→降 max_velocity 到 10 或收 lead_cap 到 0.20；底部仍反弹→加大 bottom_soft_rad。

- 2026-07-05: [收拍三路同步·A+B+C] 用户否定俯仰是主因，指出卡顿=**三电机速度不匹配**（有的正有的已负）。CSV 印证：中段 m2 一路慢 12-15%；底部 t=0.220→0.225 内三路从(-8.6,-9.0,-8.4)变(-0.14,-5.2,-1.6)——不同时撞机械底、平台偏摆。根因：三路发**完全相同的开环轨迹**、规划跑实机前 0.4rad(terr 全程-0.4)、各自饱和"各跑各的"、收拍段无 tracking_pause、到底规划已在-0.35 硬拉过零。改 FAST_RETRACT 为**三路共享一条 logical 轨迹**：①B 减速依据=最慢那路的**实际位置**闭环(`group_err=0-max_actual`)，落后者没下来就一起等；②A 领先量钳位`lead_cap_rad=0.10`——规划不许比最慢实际低超过此值→kp 变同步器；③C 底部软着陆`bottom_soft_rad=0.15`——实际角贴零撤下向下前馈+规划不过零，先到的贴零等其他两路。`enterFastRetract` 共享轨迹从最高那路起步。config retract 段加 `lead_cap_rad/bottom_soft_rad`。验证：`colcon build` 通过。**待上电抓 RETRACT_CSV 看三路 av 是否收齐、底部是否不再暴冲**；同步太弱调小 lead_cap，拽回太慢调大。顶端+7.8冲入悬停仍未单独处理（同步后若仍顿再说）。

- 2026-07-05: [俯仰抬起梯形规划·去卡顿] 读新 retract.csv（8 发）确认：俯仰超时已修（全 `到位=是`），但卡顿源锁定=**俯仰抬起冲击**——`tilt_vel` 每发都在 t≈0.045s 从 0 一步窜到 ~6rad/s（近乎冲击），8 发一致；delta 顶部过冲反而小/不稳定。根因：上一步为修超时把俯仰调猛（kp2.0+放宽钳位+ff1.1），`tiltCommandDown` 又是阶跃甩终点角 → 电机瞬时满力矩窜起 → 反作用砸机架耦合进 delta。算法改：`tiltCommandDown()`→`tiltCommandRaise(dt)`，抬起改**梯形规划**（限加速度爬升 + velocity 前馈 + 抬起重力前馈），命令角按 `raise_rate_rad_s`/`raise_accel_rad_s2`/`raise_p_gain` 有界爬升，电机只看小误差、加速度摊开、速度仍到位。4 处抬起态调用改 `tiltCommandRaise(dt)`；`enterFastRetract` 加 `tilt_cmd_vel_=0` 复位。`arm_config.yaml` tilt 段加 `raise_rate_rad_s=6.0/raise_accel_rad_s2=40.0/raise_p_gain=25.0`。验证：`colcon build` 通过。**待上电抓 RETRACT_CSV 看 tilt_vel 是否不再瞬时窜 6、卡顿是否消失**；仍冲击→调小 raise_accel，太慢→调大。**未修**：delta 顶部过冲反向（allMotorsReached 只看位置）。

- 2026-07-05: [俯仰方向厘清 + 回摆前馈修复] 用户指出俯仰实际是**向上抬起**(0=躺平→1.6=抬到击球角)，非"下压"(历史命名 down_angle 沿用)。查驱动 `unitree_motor_native.cpp:99-101`：力矩/位置同号无翻转，`direction=1`，CSV 印证命令 1.6 时 pos/vel 均为正=向上。用户确认松力矩臂会掉回 0→重力往 0(下)拽→保持抬起需 **+(向上)** 前馈托住，`torque_ff` 正号正确（用户已把它 0.3→1.1）。**副作用修复**：`torque_ff` 原在抬起(`tiltCommandDown`)和回摆(`tiltCommandReturn` 斜坡)都用；回摆(1.6→0)重力帮忙，+1.1 向上前馈会顶着臂下不来（0.3 时轻微，1.1 时明显）。改 `tiltCommandReturn` 回摆全程改用 `tilt_ready_torque_ff_`(=0)，让重力带回、软增益控速。同步改 `arm_config.yaml` tilt 段注释（下压→抬起，标注方向约定）。验证：`colcon build` 通过。待上电验证俯仰抬起能到位保持、回摆能顺畅躺回。

- 2026-07-05: [俯仰下压超时修复] 读 retract.csv 定位：①软着陆全程 cmd_tq=0、三路位置冻结→**软着陆无软件向下力**（用户"刚开始的向下力"不在此阶段）；②收拍最大顿挫是**顶部过冲反向**——`allMotorsReached` 只看位置不看速度，臂以 +8rad/s 穿过目标 1.2 冲到 1.43 再猛反向到 −11rad/s；③**俯仰每次超时**：`publishTiltCommand` 的 `max_position_error=0.5` 钳位使力矩上限只有 `kp×0.5=0.5Nm`，tilt_vel 卡在 1.7rad/s，1.6rad 到不了。用户选先修俯仰超时。改 `arm_config.yaml` tilt 段：`kp` 1.0→2.0、`kd` 0.25→0.30、`max_position_error` 0.5→0.9（力矩上限 0.5→1.8Nm，3.6x）；`serve_test.tilt_timeout_s` 0.5→0.7。验证：`colcon build` 通过。**待上电验证俯仰是否不再超时**；若仍超时→1.6rad 超出机械行程，需降 `down_angle_rad`。**未修**：顶部过冲反向（allMotorsReached 加速度判据）、俯仰-delta 耦合——用户后续再定。

- 2026-07-04: [软着陆向下力数据记录] 用户怀疑"刚开始有一个向下的力"，要求记录定位。确认软着陆阶段三路 delta 只收到 `torque_ff=downward_torque_`(现=0)+`kd=0.05`(`kp=0`)，**俯仰在此阶段完全不下发命令(limp 自由)**——理论上无主动向下力，下沉应为自重。加**软着陆逐周期记录**：`arm_config.yaml` initialization 段加 `debug_log: true`；INIT→SOFT_LANDING 打印 CSV 表头，SOFT_LANDING 每周期打印 `LANDING_CSV,t,cmd_tq,landing_kd,m{1..3}_{pos,vel},tilt_{pos,vel}`。抓取：`grep LANDING_CSV`。判读：cmd_tq≈0 但 m_vel 持续为负→自重(重力)；tilt_vel 非零→俯仰在动。验证：`colcon build` 通过。待用户上电抓 CSV 回传。

- 2026-07-04: [收拍卡顿数据记录] 用户反馈 delta 卡顿出现在**发球收拍回落(FAST_RETRACT)的中间位置**。收拍是反向梯形规划(retract_max_velocity=15、planner_p_gain=25，仅最后 0.6 rad 减速)，同时 `tiltCommandDown()` 俯仰猛砸——两个疑似因：①俯仰下压耦合晃动；②收拍速度上限超电机能力。为定位，加**收拍逐周期数据记录**：`arm_config.yaml` retract 段加 `debug_log: true`；`enterFastRetract` 打印 CSV 表头，`FAST_RETRACT` 每周期打印一行 `RETRACT_CSV,t,m{1..3}_{pd,ad,terr,pv,av},tilt_{cmd,pos,vel}`（pd=规划增量 ad=实际增量 terr=跟踪误差 pv=规划速度 av=实际速度）。抓取：`ros2 run ... delta_arm_manager 2>&1 | grep RETRACT_CSV > retract.csv`。判读：卡顿时刻 terr 突增→电机跟不上(降 max_velocity)；tilt_vel 峰值同步→俯仰耦合(收拍时俯仰延后下压)。验证：`colcon build` 通过。待用户上电抓 CSV 回传定位。

- 2026-07-04: [夜间模式：上抛俯仰刚性保持] 用户反馈：上抛过程中臂"卡一下"、声音来自俯仰电机、上抛仍超时；并启用夜间模式。诊断：`motor_control_node` 串口线程对 GO8010 **持续重发最后一条命令**（无超时清零，`motor_control_node.cpp:374`），而 `EXECUTE` 状态不发俯仰命令 → 上抛全程俯仰只有 READY 留下的软回摆保持（`return_kp=0.5`），三路上抛反作用力矩来回摇动俯仰减速箱 → 齿隙撞击（异响/卡顿）→ 干扰 delta 跟踪 → ±0.05 rad 到位判定失败 → 上抛 1.0s 超时。修复：`EXECUTE` 状态每周期 `publishTiltCommand(tilt_cmd_angle_, 0, ready_torque_ff, tilt_kp_, tilt_kd_)` 用下压级强增益锁死俯仰在待机角。**假设（夜间模式）**：①"超时"指 EXECUTE 上抛超时（日志"上抛阶段超时"）；②俯仰刚性保持复用下压 kp/kd（1.0/0.25），未新增独立 hold 参数。**验证：claude-fable-5 分类器不可用，Bash 间歇被挡，`colcon build` 尚未跑成——用户上电前必须先构建。**

- 2026-07-04: [俯仰零点解耦 + 回摆稳重化] 用户反馈三问题：①初始化不想再给向下力（已自行把 `downward_torque` 设 0）；②俯仰下压要快、回摆要稳（减齿轮冲击）；③初始化仍有向下力 + 俯仰异响。诊断：**俯仰命令用的是电机上电原始坐标系（无零点捕获）**——三路 delta 有 `zero_positions_` 解耦，俯仰没有；上电位姿≠躺平时 READY 命令 0.0 会持续硬拉 → 异响 + 整臂被压（"初始化仍有向下力"的真凶），且 `torque_ff=0.3` 待机时一直顶着支撑。修复：①`motorStateCallback` 首帧在线反馈锁定 `tilt_zero_position_`，`publishTiltCommand` 改收相对角（零点未锁定前不发命令），`tiltReached` 改相对坐标；②新增 `tiltCommandDown()`（直接给击球角，快）/`tiltCommandReturn(dt)`（命令角按 `return_rate_rad_s` 斜坡滑回待机 + 软增益 `return_kp/return_kd`，到位后切 `ready_torque_ff=0`）；4 个下压态用前者，READY/RECOVER_READY 用后者；③修软着陆超时路径未捕获 `zero_positions_` 的 bug（会拿上电原点当零点硬拉）。`arm_config.yaml` tilt 段新增 `return_kp=0.5/return_kd=0.30/return_rate_rad_s=3.0/ready_torque_ff=0.0`。**验证：因 claude-fable-5 模型分类器不可用，Bash 被挡，`colcon build` 尚未执行——下轮必须先构建再上电。** 注意：上电/启动节点前俯仰必须处于躺平待机位（首帧反馈即零点）。

- 2026-07-04: [俯仰提速] 用户反馈 delta 俯仰（tilt）太慢。俯仰是纯 PD 直接给目标角（无梯形规划），速度瓶颈是 `max_position_error=0.50`——PD 误差被钳到 0.5 rad，力矩上限只有 `kp×0.5=0.5 Nm`（转子侧）。`arm_config.yaml` `tilt` 段调激进：`kp` 1.0→2.5、`kd` 0.25→0.35（补阻尼防过冲）、`max_position_error` 0.50→1.20（放宽让 PD 看到更大误差跑满力矩）。`torque_ff` 不变。验证：`colcon build --packages-select motor_control_ros2` 通过。待上电验证俯仰速度与是否过冲/砸击球位。

- 2026-07-04: [serve_windmill 角度累积修复] `motorStateCallback` 原来直接用 `msg->angle`（0~360° 归一化），导致 PD 误差永远偏向一侧、电机狂转。改为跨零检测累积：首帧直接赋值，后续帧计算 delta 并在 |delta|>180° 时补偿 ±360°，`feedback_angle_deg_` 变为真正的连续累积角度；hpp 新增 `last_raw_angle_deg_` 成员。验证：`colcon build` 通过（0 error）。

- 2026-07-04: [serve_windmill_manager 移植] 从 `/home/toe/111/USB2CAN_motor` 搬入 `serve_windmill_manager.cpp`/`.hpp`（风车发球状态机：IDLE→重力归零→引拍→开火→自由滑行→接住，DJI3508 双电机差分，PD 位置控制，飞车保护）和 `serve_windmill_params.yaml`；`CMakeLists.txt` 新增 `serve_windmill_manager` 可执行目标并加入 install。验证：`colcon build --packages-select motor_control_ros2` 通过（0 warning，0 error）。待上电联调 wind_up/fire_end 角度与电流参数。

- 2026-07-04: [收拍回落提速] 用户反馈 `FAST_RETRACT` 收拍回落太慢，速度优先。三项改动：①`publishCommand` 加 `bypass_clamp` 形参，收拍时跳过 `max_position_error` 钳位，让 PD 看到完整误差、跑满回落力矩；②`FAST_RETRACT` 从「直接甩零点位置 + vel=0 硬拉」改为反向梯形规划：`enterFastRetract` 让 `planned_deltas_rad_` 从当前顶点出发，规划器驱动回 0 并带 `velocity_target` 前馈，仅贴近零点时收速度防砸零点；用独立高速上限 `retract_max_velocity_`/`retract_max_acceleration_`；③`arm_config.yaml` retract 段调激进：`kp` 0.80→1.50、`torque_ff` -0.3→-0.5，新增 `max_velocity=15`、`max_acceleration=80`（均高于 execute 段）。随后按用户要求把上抛(`EXECUTE`)也 bypass 钳位（本身已有梯形规划+速度前馈），上抛落后时同样跑满力矩，粗跟踪由 `tracking_error_pause_` 兜底；未改 `motion_profile` 速度上限（直接影响上抛能量/球高）。验证：`colcon build --packages-select motor_control_ros2` 通过（0 warning，0 error）。待上电验证回落/上抛速度与零点是否回弹。
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
