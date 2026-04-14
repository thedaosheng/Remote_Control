# Teleop 项目工作日志

## 2026-04-04 会话记录 — ROS2 仿真验证环境搭建

---

## 一、今天做了什么（完整清单）

### 1. 环境确认与包安装
- 确认 ROS2 Humble 已装（`/opt/ros/humble`），但未在 PATH 中 → 已在 `.bashrc` 中 source
- 安装缺失的包：
  ```
  ros-humble-xacro
  ros-humble-joint-state-publisher / joint-state-publisher-gui
  ros-humble-ros2-control / ros2-controllers
  ros-humble-controller-manager
  ros-humble-forward-command-controller
  ros-humble-joint-state-broadcaster
  ros-humble-hardware-interface
  ros-humble-ros2controlcli
  python3-colcon-common-extensions
  ros-humble-rqt-graph / rqt-topic
  ```
- 修复 catkin_pkg 问题：`pip install catkin_pkg lark`（miniconda 缺这个包导致 colcon build 失败）

### 2. 工作空间建立
- 路径：`/home/rhz/teleop/ros2_ws/`
- 两个包：
  - `airbot_description`（ament_cmake）— 存 URDF/xacro/launch/config/meshes
  - `airbot_control`（ament_python）— 存控制节点 Python 脚本

### 3. URDF 双臂化
- 参考文件：`DISCOVERSE-main/models/mjcf/manipulator/airbot_play_force/_play_force.urdf`（play_g2 机器人）
- STL 网格文件来自：`DISCOVERSE-main/models/meshes/airbot_play_force/`
- 写了两个 xacro 文件：
  - `airbot_arm.xacro` — 单臂宏定义（`<xacro:macro name="airbot_arm" params="prefix parent_link xyz rpy">`）
  - `airbot_dual_arm.xacro` — 双臂整体（实例化两次宏 + ros2_control 接口）

**运动链（单臂）：**
```
${prefix}base_link
  → joint1 → link1 → joint2 → link2 → joint3 → link3
  → joint4 → link4 → joint5 → link5 → joint6 → link6
  → arm_connect_joint → eef_connect_base_link
  → connect_eef_joint → g2_base_link
  → g2_left_joint → g2_left_link
  → g2_right_joint → g2_right_link
  → joint_tcp → tcp（末端虚拟link）
```

**双臂挂载：**
```
world → torso_link（躯干长方体）
  → left_base_link  (xyz="0 0.20 0.58", rpy="0 0 0")
  → right_base_link (xyz="0 -0.20 0.58", rpy="0 0 0")  ← 镜像对称
```

### 4. ros2_control mock hardware 接入
- 在 `airbot_dual_arm.xacro` 中加入 `<ros2_control>` 标签
- 使用 `fake_components/GenericSystem`（不需要真实电机，command 直接回传 state）
- 接入 12 个关节（左右各 6 个 revolute joint），每个关节：
  - `<command_interface name="position"/>`
  - `<state_interface name="position">` + `<state_interface name="velocity"/>`
- 配置文件：`config/controllers.yaml`（controller_manager 参数）+ `config/fcc_params.yaml`（forward_command_controller 参数，`/**:` 通配符格式）

### 5. Launch 文件
- `launch/display.launch.py` — 可视化验证（robot_state_publisher + joint_state_publisher_gui + rviz2）
- `launch/control.launch.py` — ros2_control 验证（controller_manager + spawner × 2 + rviz2）
- `config/display.rviz` — 预配置 RViz2（Fixed Frame=world，自动加载 RobotModel）

### 6. 两个核心控制节点
- `teleop_bridge_node.py`：WebSocket 服务（0.0.0.0:8080）接收 VP 指令 → 发布 `/joint_commands` + `/forward_command_controller/commands`
- `motor_control_node.py`：订阅 `/joint_commands` → mock 打印 + 回显 `/joint_states_echo`

### 7. miniconda 污染修复
- 创建 `~/bin/ros2` wrapper 脚本，强制使用 `/usr/bin/python3`（系统 Python 3.10）
- 在 `.bashrc` 末尾加 `export PATH="$HOME/bin:$PATH"`
- 新终端即生效

---

## 二、怎么用（日常操作命令）

### 启动可视化验证（不需要真实硬件）
```bash
cd ~/teleop/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch airbot_description display.launch.py
# → 弹出 RViz2（双臂模型）+ joint_state_publisher_gui（滑条控制）
```

### 启动 ros2_control mock 模式
```bash
ros2 launch airbot_description control.launch.py
# → 启动 controller_manager + 两个控制器（joint_state_broadcaster + forward_command_controller）
```

### 验证控制器状态
```bash
ros2 control list_controllers
# 期望输出：
# joint_state_broadcaster    [active]
# forward_command_controller [active]
```

### 发送关节指令（测试用）
```bash
# 用 /usr/bin/python3 绕开 miniconda
/usr/bin/python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
rclpy.init()
node = Node('test_pub')
pub = node.create_publisher(Float64MultiArray, '/joint_commands', 10)
time.sleep(0.3)
msg = Float64MultiArray()
msg.data = [0.52]*6 + [-0.52]*6  # 左臂+30°, 右臂-30°
pub.publish(msg)
time.sleep(0.3)
node.destroy_node()
rclpy.shutdown()
EOF
```

### 启动两个控制节点
```bash
# 终端1
ros2 run airbot_control motor_control_node

# 终端2（VP 连上之后）
ros2 run airbot_control teleop_bridge_node
```

---

## 三、逻辑架构

### 整体数据流
```
Apple Vision Pro（手腕/手指关节角）
  ↓ WebSocket JSON {"left":[6个弧度], "right":[6个弧度]}
teleop_bridge_node（ROS2节点）
  ↓ /joint_commands (Float64MultiArray, 12个值)
  ↓ /forward_command_controller/commands (同上)
         ↓                                    ↓
motor_control_node                    ros2_control
（mock打印/后续接CAN）          controller_manager（100Hz）
                                  fake_components/GenericSystem
                                  joint_state_broadcaster
                                        ↓
                                  /joint_states (sensor_msgs/JointState)
                                        ↓
                                  robot_state_publisher
                                        ↓
                                  /tf（所有link的坐标变换）
                                        ↓
                                  RViz2（实时可视化）
```

### ROS2 与 ROS1 的关键差异（本次踩到的）
| 项目 | ROS1 | ROS2 Humble |
|------|------|-------------|
| 编译 | catkin_make | colcon build |
| 工作空间 source | source devel/setup.bash | source install/setup.bash |
| launch 文件 | .launch（XML） | .launch.py（Python） |
| 参数传递 | rosparam yaml | Node(parameters=[...]) |
| URDF 展开 | xacro file.xacro | xacro + ParameterValue(..., value_type=str) |
| mock 硬件 | gazebo_ros_control | fake_components/GenericSystem |
| mimic joint | robot_state_publisher 原生支持 | **不支持**，会导致 link 位置错乱 |

### 文件结构
```
/home/rhz/teleop/ros2_ws/src/
├── airbot_description/
│   ├── urdf/
│   │   ├── airbot_arm.xacro       ← 单臂宏（6DOF+G2夹爪）
│   │   └── airbot_dual_arm.xacro  ← 双臂+ros2_control接口
│   ├── launch/
│   │   ├── display.launch.py      ← 可视化验证
│   │   └── control.launch.py      ← ros2_control验证
│   ├── config/
│   │   ├── controllers.yaml       ← controller_manager配置
│   │   ├── fcc_params.yaml        ← forward_command_controller参数
│   │   └── display.rviz           ← RViz2预设
│   └── meshes/                    ← 11个STL文件（来自airbot_play_force）
└── airbot_control/
    └── airbot_control/
        ├── teleop_bridge_node.py  ← WebSocket→ROS2话题桥
        └── motor_control_node.py  ← 关节指令→mock执行
```

---

## 四、遗留问题 & 后续方向

### 已知问题
1. **miniconda 污染**：每次新终端需确认 `which ros2` 指向 `~/bin/ros2`
2. **PREEMPT_RT 未配置**：controller_manager 提示无法设置 FIFO 实时调度（正式部署时需要）
3. **夹爪 mimic joint**：ROS2 robot_state_publisher 不支持，暂未接 control，视觉上两指独立

### 今晚未做（已预留接口）
- Gazebo Harmonic 物理仿真
- MoveIt2 配置（moveit_servo 实时遥操作碰撞检测）
- PREEMPT_RT 实时内核（NUC11PH，目标 500Hz 控制环）
- 升降关节（torso_link 下方预留 lift_joint prismatic 接口）
- 真实 CAN 驱动（motor_control_node 中 `_send_can` 方法已预留）

---

## 2026-04-13 会话记录 — DM4310 舵轮底盘键盘控制

PR: thedaosheng/Remote_Control#3

---

### 一、今天做了什么

#### 1. 硬件调通：达妙 DM4310 × 4 舵轮电机
- 排查 USB-CAN 适配器：系统有 DISCOVER Robotics ×2、gs_usb ×1、HDSC CDC ×1
- 最终确认达妙 USB-CAN 适配器是 **HDSC CDC** 设备，端口 `/dev/ttyACM2`（重插后编号会变）
- 诊断 CAN 线缆断裂：USB 端通信正常（写入被消费、吞吐 444KB/s），但 CAN 总线无应答 → 用户修复线缆后恢复
- 4 个电机全部在线：FL=0x03, FR=0x06, RL=0x04, RR=0x05

#### 2. 电机 0x04 ERR=0x2 故障
- 电机 0x04 (RL) 能通信、能使能、能读参数，但 **不执行运动指令**
- 反馈帧 ERR 字段持续为 0x2（不在标准故障码表 0/1/8/9/A/B/C/D/E 中）
- 设零位、断电重启均无效，疑似硬件故障
- 驱动节点已做自动跳过处理

#### 3. ROS2 三节点架构
新建 `swerve_dm_driver_node` 驱动节点，与已有节点组成完整链路：
```
keyboard_teleop_node → /cmd_vel (Twist)
mujoco_sim_node      → /mujoco/swerve_cmd (Float64MultiArray[8])
swerve_dm_driver_node → USB-CAN → DM4310 × 4
```

#### 4. 关键 debug 经验
- `set_zero_position` **必须在使能之前调用**，否则电机进入异常状态不执行命令
- MIT 模式纯转向控制：`dq` 必须设为 0，不能混入驱动速度，否则电机被拉偏
- 电机位置偏移可达数百度（多圈累积），启动时必须设零或记录偏移

### 二、启动命令

```bash
# 终端 1: MuJoCo 仿真 (显示舵轮运动)
cd /home/rhz/teleop/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
DISPLAY=:0 ros2 run teleop_mujoco_sim mujoco_sim_node --ros-args -p enable_touch:=false

# 终端 2: 键盘控制
cd /home/rhz/teleop/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
DISPLAY=:0 ros2 run teleop_mujoco_sim keyboard_teleop_node

# 终端 3: 电机驱动 (注意 ttyACM 编号可能变化)
cd /home/rhz/teleop/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run teleop_mujoco_sim swerve_dm_driver_node --ros-args -p serial_port:=/dev/ttyACM2
```

键盘: WASD=全向平移, QE=原地旋转, GH=升降, IJKL=云台, R=归零, ESC=退出

### 三、文件清单

| 文件 | 说明 |
|------|------|
| `ros2_ws/src/teleop_mujoco_sim/teleop_mujoco_sim/swerve_dm_driver_node.py` | ROS2 电机驱动节点 (核心) |
| `ros2_ws/src/teleop_mujoco_sim/teleop_mujoco_sim/keyboard_teleop_node.py` | 键盘遥操作节点 |
| `scripts/20260413-cc-swerve_motor_test.py` | 电机扫描/摆动测试 |
| `scripts/20260413-cc-swerve_motor_control.py` | MuJoCo+电机独立联动 (不依赖 ROS2) |

### 四、遗留问题
1. **0x04 电机 ERR=0x2** — 硬件故障待更换/排查
2. **USB-CAN 端口漂移** — HDSC CDC 重插后 ttyACM 编号会变，需确认
3. **驱动电机未接入** — 当前只控制转向角，驱动轮速需要额外电机或同轴方案
4. **MuJoCo 联动优化** — 仿真中底盘是力驱动，真实电机是位置控制，两者反馈尚未闭环
