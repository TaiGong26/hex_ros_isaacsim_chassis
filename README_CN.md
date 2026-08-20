# hex_ros_robot_chassis
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 控制模式](#4-控制模式)
- [5. 参数说明](#5-参数说明)
- [6. 依赖关系](#6-依赖关系)
- [7. 快速使用](#7-快速使用)

---

## 1. 包的简介

这是 **HEXFELLOW** 底盘的 **ROS 驱动包**。

- **Trigger A3 LR1** 是一款三轮全向底盘，配备三个全向轮，实现平面内三自由度运动（前后、左右、旋转）。
- **Trigger A3 H1** 是与 **Trigger A3 LR1** 同底盘的 3 电机变体，区别在于电机不同：H1 支持真 **MIT 阻抗控制**，而 LR1 的 `MIT` 语义是"目标速度 + 最大限制电流"。
- **Maver** 是一款四轮转向底盘，配备 8 个电机（4 个转向关节 `joint_yaw1` ~ `joint_yaw4` + 4 个驱动关节 `joint_wheel1` ~ `joint_wheel4`），支持两种硬件变体：**X4H1**（`robot_type=30`）与 **L4H1**（`robot_type=31`）。

> A3 底盘（LR1 / H1）三个驱动关节统一命名为 `joint_1` ~ `joint_3`，与 [hex_ros_urdf_trigger_a](https://github.com/hexfellow/hex_ros_urdf_trigger_a) 的 URDF 及 [hex_ros_sim_trigger_a](https://github.com/hexfellow/hex_ros_sim_trigger_a) 仿真一致。

本包通过 WebSocket 连接底盘控制器，将 ROS 控制指令转发给硬件，并发布底盘实时状态、里程计、关节状态和 TF 变换。
支持 **ROS 1** 和 **ROS 2**。



---

## 2. 包架构

```
hex_ros_robot_chassis/
├── config/                              # 参数配置
│   ├── ros1/
│   │   ├── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 1 参数
│   │   ├── trigger_a3_h1_params.yaml    #   A3 H1 ROS 1 参数
│   │   ├── maver_params.yaml            #   Maver ROS 1 参数
│   │   └── display_maver_x4.rviz        #   Maver ROS 1 rviz 配置
│   └── ros2/
│       ├── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 2 参数
│       ├── trigger_a3_h1_params.yaml    #   A3 H1 ROS 2 参数
│       ├── maver_params.yaml            #   Maver ROS 2 参数
│       └── display_maver_x4.rviz        #   Maver ROS 2 rviz 配置
├── launch/                              # 启动文件
│   ├── ros1/
│   │   ├── trigger_a3_lr1.launch        #   A3 LR1 ROS 1 启动
│   │   ├── trigger_a3_h1.launch         #   A3 H1 ROS 1 启动（含 rviz，供 demo include）
│   │   └── maver.launch                 #   Maver ROS 1 启动（含 rviz）
│   └── ros2/
│       ├── trigger_a3_lr1.launch.py     #   A3 LR1 ROS 2 启动
│       ├── trigger_a3_h1.launch.py      #   A3 H1 ROS 2 启动（含 rviz，供 demo include）
│       └── maver.launch.py              #   Maver ROS 2 启动（含 rviz）
├── hex_ros_robot_chassis/               # 核心代码
│   ├── robot_trigger_a3_lr1.py          #   A3 LR1 主节点（控制循环 + ROS 接口）
│   ├── robot_trigger_a3_h1.py           #   A3 H1 主节点（控制循环 + ROS 接口，MIT 阻抗）
│   ├── robot_maver.py                   #   Maver 主节点（控制循环 + ROS 接口）
│   ├── utility/                         #   Trigger A3 LR1 / H1 共用的 DataInterface
│   │   ├── __init__.py                  #     根据 ROS_VERSION 自动选择 ROS1/ROS2 实现
│   │   ├── interface_base.py            #     抽象基类 ChassisInterfaceBase
│   │   ├── ros1_interface.py            #     ROS 1 DataInterface 实现
│   │   └── ros2_interface.py            #     ROS 2 DataInterface 实现
│   └── maver_util/                      #   Maver 的 DataInterface（与 utility/ 同构）
│       ├── __init__.py                  #     根据 ROS_VERSION 自动选择 ROS1/ROS2 实现
│       ├── interface_base.py            #     抽象基类 ChassisInterfaceBase
│       ├── ros1_interface.py            #     ROS 1 DataInterface 实现
│       └── ros2_interface.py            #     ROS 2 DataInterface 实现
├── resource/                            # ament 资源文件
├── setup.py                             # Python 打包配置（3 个 entry_point）
├── setup.cfg                            # Python 打包配置
├── package.xml                          # ROS 包清单（双系统条件依赖）
└── README.md                            # 英文文档
```

### 接口层说明

`utility/`（Trigger A3 LR1 / H1）与 `maver_util/`（Maver）模块各提供统一的 `DataInterface`，根据 `ROS_VERSION` 环境变量自动选择 ROS 1 或 ROS 2 实现：

| 实现 | 文件 | 适用版本 |
|------|------|---------|
| `ros1_interface.DataInterface` | `ros1_interface.py` | ROS 1（Noetic） |
| `ros2_interface.DataInterface` | `ros2_interface.py` | ROS 2（Humble / Foxy） |

> `maver_util/` 结构与 `utility/` 相同，为 Maver 专用接口层（多了 `robot_type` 参数读取）。A3 LR1 与 A3 H1 均复用 `utility/`（A3 H1 机型固定，无需 `robot_type`）。

---

## 3. 话题接口

### Trigger A3 LR1

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | 底盘控制指令（VEL / MIT 模式） |
| 发布 | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | 底盘状态反馈（3 个轮子的位置、速度、力矩） |
| 发布 | `odom` | `nav_msgs/(msg/)Odometry` | 里程计（位置 x, y, yaw + 速度 vx, vy, omega） |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 个轮关节状态（`joint_1` ~ `joint_3`） |
| 发布 | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link 坐标系变换 |

### Trigger A3 H1

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | 底盘控制指令（VEL / MIT 模式） |
| 发布 | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | 底盘状态反馈（3 个电机的的位置、速度、力矩） |
| 发布 | `odom` | `nav_msgs/(msg/)Odometry` | 里程计（位置 x, y, yaw + 速度 vx, vy, omega） |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 个关节状态（`joint_1` ~ `joint_3`） |
| 发布 | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link 坐标系变换 |

> A3 LR1 与 A3 H1 发布相同的话题与关节名（`joint_1` ~ `joint_3`），区别仅在驱动电机与控制语义（见「4. 控制模式」）。

### Maver

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | 底盘控制指令（VEL / MIT 模式） |
| 发布 | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | 底盘状态反馈（8 个电机的位置、速度、力矩） |
| 发布 | `odom` | `nav_msgs/(msg/)Odometry` | 里程计（位置 x, y, yaw + 速度 vx, vy, omega） |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 8 个关节状态（`joint_wheel1`, `joint_yaw1`, ..., `joint_wheel4`, `joint_yaw4`） |
| 发布 | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link 坐标系变换 |

> **关节顺序**：Maver 的 8 个关节（数组索引 0~7）按 `joint_wheel1, joint_yaw1, joint_wheel2, joint_yaw2, joint_wheel3, joint_yaw3, joint_wheel4, joint_yaw4` 排列；其中索引 0, 2, 4, 6 为驱动（wheel）关节，索引 1, 3, 5, 7 为转向（yaw）关节。`chs_ctrl` 的 `jnt` 数组与 `joint_states` 均须遵守该顺序。

> 消息类型定义见 [hex_ros_msgs](https://github.com/hexfellow/hex_ros_msgs)

> 默认提供ros时间；如果您需要硬件时间戳，可以通过`chs_state.jnt.header.stamp`获取


---

## 4. 控制模式

底盘支持两种控制模式，通过 `chs_ctrl.ctrl_mode` 选择：

| 模式 | 值 |  说明 |
|------|-----|------|
| `VEL` | `2` |  速度模式：下发 (vx, vy, omega) 三自由度速度指令 |
| `MIT` | `1` | 力矩模式：语义随电机驱动而异（见下方各机型说明） |
| `NONE` | `0` | 空操作，不执行任何控制 |

> **Trigger A3 LR1**：MIT 模式下发**目标速度 (rad/s) + 最大限制电流 (A)**，映射到 `set_chs_per_motor_spd_cmd`，并非真阻抗控制。

> **Trigger A3 H1**：MIT 模式为**真阻抗控制**，通过 `set_chs_mit_cmd` 下发位置/速度/刚度/阻尼；当前固件强制 `kp=0`。

> **Maver** 当前固件强制 `kp=0`。

### MIT 模式使用警告

> **MIT 模式使用警告**：除非你知道什么是 MIT 模式，否则不要使用该模式；

> 使用不当可能导致底盘剧烈运动甚至损坏设备。

> 确保在安全区域内运行，并随时准备急停。

---

## 5. 参数说明

### Trigger A3 LR1

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_state` | 500.0 | 状态发布频率（从 ctrl_rate 降采样）[Hz] |
| `robot_host` | 192.168.1.100 | 底盘控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `enable_kcp` | `true` | 是否启用 KCP 传输协议 |

### Trigger A3 H1

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_state` | 500.0 | 状态发布频率（从 ctrl_rate 降采样）[Hz] |
| `robot_host` | 192.168.1.100 | 底盘控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `sens_ts` | `true` | 是否使用硬件传感器时间戳 |
| `enable_kcp` | `true` | 是否启用 KCP 传输协议 |

> A3 H1 与 LR1 参数相同（无 `robot_type`，机型固定为 H1）。

### Maver 

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_state` | 500.0 | 状态发布频率 [Hz] |
| `robot_host` | 192.168.1.100 | 底盘控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `sens_ts` | `true` | 是否使用硬件传感器时间戳 |
| `enable_kcp` | `true` | 是否启用 KCP 传输协议 |
| `robot_type` | 30 | 机型：30=X4H1，31=L4H1 |

> `rate_state`（状态发布频率）默认 500.0。注：ROS2 config `config/ros2/maver_params.yaml` 中该值为 1000.0，以 500.0 为准。

---

## 6. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0a4'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

> A3 底盘（LR1 / H1）的 rviz 可视化（launch `rviz:=true`）需要额外的 URDF 包：

```shell
git clone https://github.com/hexfellow/hex_ros_urdf_trigger_a.git
```

> Maver 的 rviz 可视化（launch `rviz:=true`）需要额外的 URDF 包：

```shell
git clone https://github.com/hexfellow/hex_ros_urdf_maver_x4.git
```

---

## 7. 快速使用

### 1. 创建工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
git clone https://github.com/hexfellow/hex_ros_urdf_trigger_a.git    # A3 底盘（LR1 / H1）
git clone https://github.com/hexfellow/hex_ros_urdf_maver_x4.git     # Maver X4
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash --extend
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash --extend
```

### 4. 使用包

#### ROS1
```shell
# ROS 1 — Trigger A3 LR1
roslaunch hex_ros_robot_chassis trigger_a3_lr1.launch \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 1 — Trigger A3 H1
roslaunch hex_ros_robot_chassis trigger_a3_h1.launch \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 1 — Maver
roslaunch hex_ros_robot_chassis maver.launch \
    robot_host:=192.168.1.100 robot_port:=8439
```

#### ROS2
```shell
# ROS 2 — Trigger A3 LR1
ros2 launch hex_ros_robot_chassis trigger_a3_lr1.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 2 — Trigger A3 H1
ros2 launch hex_ros_robot_chassis trigger_a3_h1.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 2 — Maver
ros2 launch hex_ros_robot_chassis maver.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439

```


`trigger_a3_h1.launch.py` / `maver.launch.py` 可选参数：
- `rviz:=true/false`：是否启动 rviz 可视化（默认 `true`，A3 需要 `hex_ros_urdf_trigger_a`，Maver 需要 `hex_ros_urdf_maver_x4`）

**Maver 机型选择：**

Maver 有 **X4H1** 与 **L4H1** 两种机型，通过 `robot_type` 参数选择（默认 `30` = X4H1）：

| 机型 | `robot_type` |
|------|--------------|
| X4H1 | `30` |
| L4H1 | `31` |

机型通过修改对应 ROS 版本的参数配置文件选择，然后启动：

- **ROS 2**：编辑 `config/ros2/maver_params.yaml` 中的 `robot_type`，再启动 `maver.launch.py`
- **ROS 1**：编辑 `config/ros1/maver_params.yaml` 中的 `robot_type`，再启动 `maver.launch`

> 将 `robot_host` 和 `robot_port` 替换为实际底盘控制器的 IP 和端口。

### 5. 控制底盘

#### Trigger A3 LR1 快速使用

通过 `ros2 topic pub` 可快速向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# VEL 模式 — 停止 
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# 目标速度最大电流限制 模式 — 三个电机同速 0.3，电流限制 2.0
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# 目标速度最大电流限制 模式 — 停止
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

```

通过`rostopic pub`可快速向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# VEL 模式 — 停止
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# 目标速度最大电流限制模式 — 三个电机同速 0.3，电流限制 2.0
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# 目标速度最大电流限制模式 — 停止
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> 目前**Trigger A3 lr**未支持MIT；当您使用MIT模式时，将会向****Trigger A3 lr****设备下发目标速度(rad/s)+最大限制电流(A)

#### Trigger A3 H1 快速使用

通过 `ros2 topic pub` 可快速向底盘发送控制指令（3 个电机按 `joint_1` ~ `joint_3` 顺序排列）：

```bash
# VEL 模式 — 旋转 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# MIT 模式 — 阻尼运动（目标速度 0.5 rad/s，阻尼 3.0，kp 当前固件强制为 0）
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# MIT 模式 — 松手（全零，无输出力矩）
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0], vel: [0, 0, 0], eff: [0, 0, 0], kp: [0, 0, 0], kd: [0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

通过 `rostopic pub`（ROS 1）可向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# MIT 模式 — 阻尼运动（目标速度 0.5 rad/s，阻尼 3.0，kp 当前固件强制为 0）
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# MIT 模式 — 松手（全零，无输出力矩）
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0], vel: [0, 0, 0], eff: [0, 0, 0], kp: [0, 0, 0], kd: [0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> A3 H1 的 MIT 为真阻抗控制（`set_chs_mit_cmd`），字段与 Maver 一致：`pos` 目标位置、`vel` 目标速度、`kp` 位置刚度、`kd` 阻尼；当前固件强制 `kp=0`。

#### Maver 快速使用

通过 `ros2 topic pub` 可快速向底盘发送控制指令（8 个电机按 `joint_wheel1, joint_yaw1, ..., joint_wheel4, joint_yaw4` 顺序排列）：

```bash
# VEL 模式 — 前进 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# VEL 模式 — 原地旋转 0.5 rad/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# MIT 模式 — 阻尼运动（目标速度 0.5 rad/s，阻尼 3.0，kp 当前固件强制为 0）
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# MIT 模式 — 松手（全零，无输出力矩）
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0, 0, 0, 0, 0, 0], vel: [0, 0, 0, 0, 0, 0, 0, 0], eff: [0, 0, 0, 0, 0, 0, 0, 0], kp: [0, 0, 0, 0, 0, 0, 0, 0], kd: [0, 0, 0, 0, 0, 0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

通过 `rostopic pub`（ROS 1）可向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}}}"

# MIT 模式 — 阻尼运动（目标速度 0.5 rad/s，阻尼 3.0，kp 当前固件强制为 0）
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# MIT 模式 — 松手（全零，无输出力矩）
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0, 0, 0, 0, 0, 0], vel: [0, 0, 0, 0, 0, 0, 0, 0], eff: [0, 0, 0, 0, 0, 0, 0, 0], kp: [0, 0, 0, 0, 0, 0, 0, 0], kd: [0, 0, 0, 0, 0, 0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

MIT 模式字段说明：

| `chs_ctrl.jnt` 字段 | 含义 | 单位 |
|------|------|------|
| `jnt.pos[0..7]` | 目标位置 | rad |
| `jnt.vel[0..7]` | 目标速度 | rad/s |
| `jnt.kp[0..7]` | 位置刚度 | Nm/rad |
| `jnt.kd[0..7]` | 阻尼 | Nm/(rad/s) |

> `--once` 发布一次只在单个控制周期内生效；如需持续运动请使用 `--rate` 定期发布。
> `robot_type` 机型选择见上文「4. 使用包」（30=X4H1 / 31=L4H1）。
> 8 个电机的数组顺序为 `joint_wheel1, joint_yaw1, joint_wheel2, joint_yaw2, joint_wheel3, joint_yaw3, joint_wheel4, joint_yaw4`（索引 0, 2, 4, 6 = wheel 驱动，1, 3, 5, 7 = yaw 转向）。