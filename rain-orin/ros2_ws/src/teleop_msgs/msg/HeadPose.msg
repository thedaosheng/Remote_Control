# 物理含义命名的头部姿态 (相对 VP 进入 Immersive 那一刻的零点)
#
# 设计:
#   - 字段名是 *物理含义* (PITCH 点头 / YAW 转头 / ROLL 侧倾),
#     不混入数学命名 (绕X/绕Y/绕Z) 避免歧义
#   - 单位是 *度* (deg), 跟 sender UDP 转发的协议一致
#   - 加 timestamp 让消费者能算延迟和判断是否过期

std_msgs/Header header

# 物理 RPY (deg, 相对零点)
float64 pitch_deg     # 点头 (抬头为正)
float64 yaw_deg       # 转头 (左转为正)
float64 roll_deg      # 侧倾 (右耳贴肩为正?)

# 头部位置 (m, 来自 VP 的 originFromAnchor)
float64 pos_x
float64 pos_y
float64 pos_z

# 四元数 (备用, vp 端的原始 quat 相对零点的旋转)
float64 quat_x
float64 quat_y
float64 quat_z
float64 quat_w
