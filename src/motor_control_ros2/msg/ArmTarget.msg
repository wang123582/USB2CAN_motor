std_msgs/Header header

# 目标相对增量（3路 GO8010 电机，弧度）
# 解耦模式要求三路值相同，表示统一 Δrad
float64[3] target_angles

# 击球俯仰角：相对压底位姿的**物理倾角增量**（弧度，正=抬头），节点内查杠杆标定表换算电机角。
# 有效域：压底位≈18°，绝对倾角钳位到标定域 [7°, 40°]，即本字段约 [-0.19, +0.38] rad。
# 先抬俯仰到该角、到位确认后才上抛击球；0.0 = 不抬（保持压底躺平位）
float64 tilt_angle_rad

# 执行标志：true 时进入 EXECUTE 状态，false 时忽略
bool execute
