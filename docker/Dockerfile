# ==============================================================
#  Teleop 舵轮底盘控制 Docker 镜像
# ==============================================================
#
# 包含:
#   - ROS2 Humble (ros-base)
#   - 达妙 DM_CAN 库 (DM4310 转向电机)
#   - ZLAC8015D CANopen 驱动 (驱动轮)
#   - MuJoCo 仿真 (可选，headless)
#   - 键盘控制 + 云服务器控制接口 (预留)
#
# 构建:
#   docker build -t teleop-swerve:latest -f docker/Dockerfile .
#
# 运行:
#   docker compose -f docker/docker-compose.yaml up

FROM ros:humble-ros-base-jammy

# 避免交互式安装提示
ENV DEBIAN_FRONTEND=noninteractive

# ---- 系统依赖 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ---- Python 依赖 ----
RUN pip3 install --no-cache-dir \
    numpy \
    pyserial \
    python-can \
    mujoco

# ---- 拷贝达妙 DM_CAN 库 ----
COPY DM_Control_Python/DM_CAN.py /opt/dm_can/DM_CAN.py
COPY DM_Control_Python/requirements.txt /opt/dm_can/requirements.txt

# ---- 拷贝 ROS2 工作空间 ----
# 只拷贝 teleop_mujoco_sim 包 (不含 build/install/log)
COPY ros2_ws/src/teleop_mujoco_sim /ros2_ws/src/teleop_mujoco_sim

# ---- 拷贝 MuJoCo 模型 ----
COPY RemoteControl/swerve_chassis /opt/mujoco_models/swerve_chassis

# ---- 编译 ROS2 包 ----
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select teleop_mujoco_sim && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=/opt/dm_can:\$PYTHONPATH" >> /root/.bashrc

# ---- 环境变量 ----
# DM_CAN 库路径
ENV PYTHONPATH="/opt/dm_can:${PYTHONPATH}"
# MuJoCo headless 渲染
ENV MUJOCO_GL=egl

# ---- 入口脚本 ----
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# 默认启动驱动节点
CMD ["driver"]
