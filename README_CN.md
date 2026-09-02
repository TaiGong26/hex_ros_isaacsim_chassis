# hex_ros_isaacsim_chassis
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 控制模式](#4-控制模式)
- [5. 参数说明](#5-参数说明)
- [6. 依赖关系](#6-依赖关系)
- [7. Isaacsim Action Graph](#7-isaacsim-action-graph)
- [8. 快速使用](#8-快速使用)
- [9. 常见问题](#9-常见问题)

---

## 1. 包的简介

`hex_ros_isaacsim_chassis` 是 Isaac Sim 底盘的 ROS 2 Bridge 转发包。

本包负责：

- 订阅上游发布的 `chs_ctrl` 底盘控制指令；
- 订阅 Isaac Sim Bridge 发布的底盘 `JointState`；
- 将支持的底盘控制指令转发为 Isaac Sim Bridge 使用的 `JointState` 命令；
- 通过参数指定状态话题和命令话题。

本包包含以下节点：

- **Maver X4**：四轮转向底盘；
- **Trigger A3**：三轮底盘。

本包不负责发布 `joint_state`、`odom` 或 `/tf`。这些话题由 Isaac Sim 场景提供，不属于本包的发布接口。

---

## 2. 包架构

```text
hex_ros_isaacsim_chassis/
├── config/ros2/
│   ├── maver_x4_params.yaml
│   └── trigger_a3_params.yaml
├── launch/ros2/
│   ├── isaacsim_maver_x4.launch.py
│   └── isaacsim_trigger_a3.launch.py
├── hex_ros_isaacsim_chassis/
│   ├── isaacsim_maver_x4.py
│   ├── isaacsim_trigger_a3.py
│   ├── maver_util/
│   └── utility/
├── package.xml
├── setup.py
└── README_CN.md
```

### 节点入口

| 节点 | 可执行文件 | 作用 |
|---|---|---|
| Maver X4 | `isaacsim_maver_x4` | 转发 Maver X4 底盘控制指令 |
| Trigger A3 | `isaacsim_trigger_a3` | 转发 Trigger A3 底盘控制指令 |

---

## 3. 话题接口

### 控制输入

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `chs_ctrl` | `hex_ros_msgs/msg/HexRosRoboChsCtrlStamped` | 上游底盘控制指令 |

### Isaac Sim Bridge 接口

| 方向 | 参数 | 默认话题 | 类型 | 说明 |
|---|---|---|---|---|
| 订阅 | `joint_state_topic` | `/joint_states` | `sensor_msgs/msg/JointState` | Isaac Sim Bridge 发布的关节状态 |
| 发布 | `joint_command_topic` | `/joint_command` | `sensor_msgs/msg/JointState` | 发往 Isaac Sim Bridge 的关节命令 |

`joint_state_topic` 和 `joint_command_topic` 都可以在对应的 YAML 参数文件中自定义。修改后，Isaac Sim Bridge 的发布/订阅话题必须与参数保持一致。

### Maver X4 关节名称

```text
joint_wheel1
joint_yaw1
joint_wheel2
joint_yaw2
joint_wheel3
joint_yaw3
joint_wheel4
joint_yaw4
```

`JointState` 的数组字段必须与同一位置的 `name` 对应。Maver 的控制数组遵循以下标准顺序：

```text
wheel1, yaw1, wheel2, yaw2, wheel3, yaw3, wheel4, yaw4
```

### Trigger A3 关节名称

```text
joint_1
joint_2
joint_3
```

---

## 4. 控制模式

控制模式由 `chs_ctrl.ctrl_mode` 指定。

| 模式 | 值 | 状态 | 说明 |
|---|---:|---|---|
| `NONE` | `0` | 支持 | 空模式，清除当前保留控制指令 |
| `MIT` | `1` | 不支持 | 输出 warning，不发布该 MIT 控制命令 |
| `VEL` | `2` | 支持 | 接收底盘线速度和角速度指令 |

Maver X4 和 Trigger A3 的 Isaac Sim Bridge 节点均不支持 MIT。收到 MIT 后不会自动转换为 VEL。

`NONE` 是空模式，不等同于发送一条零速度的 `VEL` 指令，也不等同于发送零位置或零力矩控制。它用于清除当前保留的控制指令。

---

## 5. 参数说明

### Maver X4

参数文件：`config/ros2/maver_x4_params.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ctrl_rate` | `1000.0` | 控制命令转发频率 [Hz] |
| `use_sim_time` | `true` | 是否使用 ROS 仿真时间 |
| `joint_state_topic` | `/joint_states` | 可自定义的 Bridge 状态话题 |
| `joint_command_topic` | `/joint_command` | 可自定义的 Bridge 命令话题 |

### Trigger A3

参数文件：`config/ros2/trigger_a3_params.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ctrl_rate` | `1000.0` | 控制命令转发频率 [Hz] |
| `use_sim_time` | `true` | 是否使用 ROS 仿真时间 |
| `joint_state_topic` | `/joint_states` | 可自定义的 Bridge 状态话题 |
| `joint_command_topic` | `/joint_command` | 可自定义的 Bridge 命令话题 |

> 如果`use_sim_time`为True，你需要在isaacsim 开启 **clock** 话题

---

## 6. 依赖关系

### Python 包

本包使用 ROS 2 Python 环境和以下 Python 依赖：

```shell
pip3 install 'hex-util-msg>=0.1.0a4'
```


### ROS 包

创建 ROS 2 工作空间并获取消息包：

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_isaacsim_chassis.git
```

### Isaac Sim ROS 2 Bridge

Isaac Sim 侧需要启用 ROS 2 Bridge，并创建与本包参数一致的 `JointState` 状态发布和命令订阅接口。安装和启用方式请参考 Isaac Sim 官方文档：

- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)

Isaac Sim 场景还需要对应的 USD 资源。请获取以下 USD 资源仓库，并按照仓库说明配置资产路径：

```shell
git clone https://github.com/hexfellow/hex_isaac_usd.git
```

---

## 7. Isaacsim Action Graph

![Action Graph](./img/80d5a0cb2d9ced03a2dc84bb76663a84.png)

---

## 8. 快速使用

### 1. 创建和编译工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_isaacsim_chassis.git

cd <your_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash --extend
```

### 2. 启动节点

启动 Maver X4：

```shell
ros2 launch hex_ros_isaacsim_chassis isaacsim_maver_x4.launch.py
```

启动 Trigger A3：

```shell
ros2 launch hex_ros_isaacsim_chassis isaacsim_trigger_a3.launch.py
```

### 3. 检查接口

```shell
ros2 node list
ros2 topic list -t
ros2 topic info /chs_ctrl -v
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
ros2 topic echo /joint_states --once
ros2 topic echo /joint_command --once
```

### 4. 发布控制指令

以下示例使用 `VEL` 模式，并持续发布指令。`/chs_ctrl` 是本包固定订阅的话题；Bridge 状态和命令话题由参数指定。停止发布者后，如需清除节点保留的控制指令，请发送 `NONE`。

```shell
# Maver: vx = 1.0 m/s
ros2 topic pub --rate 10 /chs_ctrl \
  hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

```shell
# Maver: wz = 1.0 rad/s
ros2 topic pub --rate 10 /chs_ctrl \
  hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}}}'
```

```shell
# Clear the retained chassis command
ros2 topic pub --once /chs_ctrl \
  hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 0}}'
```

### 5. 自定义 Bridge 话题

在参数文件中修改：

```yaml
joint_state_topic: "/my_isaac_joint_states"
joint_command_topic: "/my_isaac_joint_command"
```

启动节点时使用该参数文件。Isaac Sim Bridge 侧也必须使用相同的话题名称。

---

## 9. 常见问题

### 没有收到 `/joint_states`

检查：

```shell
ros2 topic info /joint_states -v
ros2 topic echo /joint_states --once
```

如果没有实际消息，请检查 Isaac Sim ROS 2 Bridge 是否已启用，以及 ROS domain、RMW、DDS discovery 和 Docker 网络是否一致。

### MIT 被拒绝

这是当前设计。Isaac Sim chassis 节点只支持：

```text
VEL：底盘速度控制
NONE：空模式，清除当前保留指令
```

收到 MIT 后会输出 warning，并且不会发布该控制命令。

### `NONE` 的含义

`NONE` 是空模式。它用于清除当前保留控制指令，不是一次新的零速度 `VEL` 指令。

### `/joint_command` 没有消息

检查：

```shell
ros2 topic info /joint_command -v
ros2 topic echo /joint_command --once
```

Maver 节点还需要有效的 `joint_state_topic` 状态。检查状态话题中的八个关节名称是否完整，并确认参数中的命令话题与 Isaac Sim Bridge 订阅话题一致。

### ROS 2 跨容器无法通信

确认两端配置一致：

```text
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
DDS discovery
Docker network
```

跨容器通信时不要使用只允许本机回环接口的配置。
