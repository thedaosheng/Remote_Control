# 双臂关节指令消息
# positions: 14个关节位置（左臂7+右臂7），单位 rad
# velocities: 14个关节速度，单位 rad/s
# mode: 控制模式字符串（idle/teleop/autonomous/estop）

float64[] positions
float64[] velocities
string mode
