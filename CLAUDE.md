# Teleop 项目工作日志

## 2026-04-09 会话记录 — Touch 力反馈笔力渲染能力探索

---

### Touch 设备关键信息（Agent 必读）

**设备**: 3D Systems Touch (Phantom Omni USB), VID:PID=2988:0302
**串口**: `/dev/ttyACM2` (Channel=2 in config, **每次拔插可能变！**)
**配置**: `~/.3dsystems/config/Default Device.config` → `Channel=2`
**补丁库**: `/tmp/patched_lib/libPhantomIOLib42.so` (重启后丢失，需重建)
**假库**: `/tmp/fakelibs/libncurses.so.5` (重启后丢失，需重建)

### 编译运行 Touch 程序的标准流程

```bash
# 1. 重建补丁库（重启后必做）
mkdir -p /tmp/patched_lib /tmp/fakelibs
gcc -shared -fPIC -o /tmp/fakelibs/libncurses.so.5 -x c /dev/null 2>/dev/null || true
python3 -c "
import shutil
shutil.copy2('/usr/lib/libPhantomIOLib42.so', '/tmp/patched_lib/libPhantomIOLib42.so')
with open('/tmp/patched_lib/libPhantomIOLib42.so', 'r+b') as f:
    f.seek(0x3f2c2); f.write(bytes([0x45,0x31,0xe4,0x90,0x90,0x90,0x90]))
    f.seek(0x32446); f.write(bytes([0x90,0x90,0x90,0x90,0x90,0x90]))
    f.seek(0x3b166); f.write(bytes([0x90,0x90,0x90,0x90,0x90,0x90]))
print('补丁重建完成')
"

# 2. 确认设备 Channel（找到 VID=2988 的 ttyACM 号）
for d in /dev/ttyACM*; do
  [ -L "$d" ] && continue
  vid=$(udevadm info --name="$d" 2>/dev/null | grep ID_VENDOR_ID | sed 's/.*=//')
  [ "$vid" = "2988" ] && echo "Touch 在 $d" && break
done
# 然后修改 ~/.3dsystems/config/'Default Device.config' 的 Channel=N

# 3. 编译（示例）
gcc -o /tmp/my_touch_app my_touch_app.c -ldl -lm

# 4. 运行
LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/my_touch_app
```

### 已知坑（踩过的）

1. **`timeout` 杀进程会锁死设备** — 必须用 `kill -2 (SIGINT)` 优雅退出，否则下次 `hdInitDevice` 返回 `0xFFFFFFFF` (0x303 错误)。解法：拔插 USB。
2. **Channel 号不固定** — 每次拔插 USB，ttyACM 号可能变。必须检查 VID=2988 在哪个 ttyACM 上。
3. **Python ctypes 调 HD API 会 core dump** — `hdGetError` 返回结构体不是 int，ctypes 无法正确处理。必须用 C 的 dlopen/dlsym 方式。
4. **补丁库在 /tmp** — 重启丢失，每次重启需重建。
5. **hdStartScheduler 返回 0x103** — 正常现象（RTAI 硬件定时器不可用），伺服线程用 clock_nanosleep 软定时，照常 ~1kHz。

### 已完成的文件

```
touch/
├── 20260409-cc-README.md                    # 总览报告（10种力渲染模式+能力边界）
├── 20260409-cc-handoff_to_next_agent.md     # Agent B 交接文档
├── demos/
│   ├── 20260409-cc-force_rendering_8modes.c # ★ 8种力渲染真机demo (已验证通过)
│   │   编译: gcc -o /tmp/force_8modes demos/20260409-cc-force_rendering_8modes.c -ldl -lm
│   │   运行: LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/force_8modes
│   │   操作: 按 0-8 切换效果, q 退出
│   │   日志: /tmp/force_8modes.log
│   ├── 01-08_basic_shapes~gravity_well/     # 各效果说明 README
│   └── 20260409-cc-touch_hd_driver.py       # Python 驱动层 (参考，不可直接用)
├── mujoco_sim/                              # 4个 MuJoCo 仿真脚本 (44张截图已生成)
├── survey/                                  # 4份调研文档
├── third_party/                             # 4个项目评估
└── screenshots/                             # 44张截图
```

---

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
