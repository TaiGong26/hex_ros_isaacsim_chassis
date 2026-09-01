# hex_ros_isaacsim_chassis
**English** | [中文](README_CN.md)

## Table of Contents

- [1. Package Overview](#1-package-overview)
- [2. Package Structure](#2-package-structure)
- [3. Topic Interface](#3-topic-interface)
- [4. Control Modes](#4-control-modes)
- [5. Parameters](#5-parameters)
- [6. Dependencies](#6-dependencies)
- [7. Isaac Sim Action Graph](#7-isaac-sim-action-graph)
- [8. Quick Start](#8-quick-start)
- [9. Common Issues](#9-common-issues)

---

## 1. Package Overview

`hex_ros_isaacsim_chassis` is a ROS 2 Bridge forwarding package for the Isaac Sim chassis.

This package:

- subscribes to upstream `chs_ctrl` chassis control commands;
- subscribes to chassis `JointState` messages published by the Isaac Sim Bridge;
- forwards supported chassis control commands as `JointState` messages for the Isaac Sim Bridge;
- uses parameters to select the state and command topics.

The package contains the following nodes:

- **Maver X4**: four-wheel-steering chassis;
- **Trigger A3**: three-wheel chassis.

This package does not publish `joint_state`, `odom`, or `/tf`. These topics are provided by the Isaac Sim scene and are not package outputs.

---

## 2. Package Structure

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

### Node Entry Points

| Node | Executable | Description |
|---|---|---|
| Maver X4 | `isaacsim_maver_x4` | Forwards Maver X4 chassis control commands |
| Trigger A3 | `isaacsim_trigger_a3` | Forwards Trigger A3 chassis control commands |

---

## 3. Topic Interface

### Control Input

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribe | `chs_ctrl` | `hex_ros_msgs/msg/HexRosRoboChsCtrlStamped` | Upstream chassis control command |

### Isaac Sim Bridge Interface

| Direction | Parameter | Default Topic | Type | Description |
|---|---|---|---|---|
| Subscribe | `joint_state_topic` | `/joint_states` | `sensor_msgs/msg/JointState` | Joint state published by the Isaac Sim Bridge |
| Publish | `joint_command_topic` | `/joint_command` | `sensor_msgs/msg/JointState` | Joint command sent to the Isaac Sim Bridge |

Both `joint_state_topic` and `joint_command_topic` can be customized in the corresponding YAML parameter file. After changing them, the Isaac Sim Bridge publisher and subscriber must use the same topic names.

### Maver X4 Joint Names

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

Every `JointState` array field must correspond to the joint name at the same index. Maver control arrays use the following standard order:

```text
wheel1, yaw1, wheel2, yaw2, wheel3, yaw3, wheel4, yaw4
```

### Trigger A3 Joint Names

```text
joint_1
joint_2
joint_3
```

---

## 4. Control Modes

The control mode is selected by `chs_ctrl.ctrl_mode`.

| Mode | Value | Status | Description |
|---|---:|---|---|
| `NONE` | `0` | Supported | Empty mode; clears the currently retained control command |
| `MIT` | `1` | Unsupported | Logs a warning and does not publish the MIT command |
| `VEL` | `2` | Supported | Receives chassis linear and angular velocity commands |

The Isaac Sim Bridge nodes for both Maver X4 and Trigger A3 do not support MIT control. MIT commands are not automatically converted to VEL commands.

`NONE` is an empty mode. It is not equivalent to sending a zero-velocity `VEL` command, a zero-position command, or a zero-torque command. It clears the currently retained control command.

---

## 5. Parameters

### Maver X4

Parameter file: `config/ros2/maver_x4_params.yaml`

| Parameter | Default | Description |
|---|---|---|
| `ctrl_rate` | `1000.0` | Control command forwarding rate [Hz] |
| `use_sim_time` | `true` | Whether to use ROS simulation time |
| `joint_state_topic` | `/joint_states` | Customizable Bridge state topic |
| `joint_command_topic` | `/joint_command` | Customizable Bridge command topic |

### Trigger A3

Parameter file: `config/ros2/trigger_a3_params.yaml`

| Parameter | Default | Description |
|---|---|---|
| `ctrl_rate` | `1000.0` | Control command forwarding rate [Hz] |
| `use_sim_time` | `true` | Whether to use ROS simulation time |
| `joint_state_topic` | `/joint_states` | Customizable Bridge state topic |
| `joint_command_topic` | `/joint_command` | Customizable Bridge command topic |

> When `use_sim_time` is `true`, the `/clock` topic must be enabled in Isaac Sim.

---

## 6. Dependencies

### Python Packages

This package uses the ROS 2 Python environment and the following Python dependency:

```shell
pip3 install 'hex-util-msg>=0.1.0a4'
```

### ROS Packages

Create a ROS 2 workspace and obtain the message and Isaac Sim chassis package repositories:

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_isaacsim_chassis.git
```

### Isaac Sim ROS 2 Bridge

The Isaac Sim side must enable the ROS 2 Bridge and provide `JointState` state publishing and command subscription interfaces that match this package's parameters. Refer to the official Isaac Sim documentation:

- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)

---

## 7. Isaac Sim Action Graph

![Action Graph](./img/80d5a0cb2d9ced03a2dc84bb76663a84.png)

---

## 8. Quick Start

### 1. Create and Build the Workspace

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

### 2. Start a Node

Start Maver X4:

```shell
ros2 launch hex_ros_isaacsim_chassis isaacsim_maver_x4.launch.py
```

Start Trigger A3:

```shell
ros2 launch hex_ros_isaacsim_chassis isaacsim_trigger_a3.launch.py
```

### 3. Check the Interfaces

```shell
ros2 node list
ros2 topic list -t
ros2 topic info /chs_ctrl -v
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
ros2 topic echo /joint_states --once
ros2 topic echo /joint_command --once
```

### 4. Publish Control Commands

The following examples use `VEL` mode and publish continuously. `/chs_ctrl` is the fixed input topic of this package; the Bridge state and command topics are selected by parameters. After stopping the publisher, send `NONE` to clear the retained command.

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

### 5. Customize Bridge Topics

Modify the parameter file:

```yaml
joint_state_topic: "/my_isaac_joint_states"
joint_command_topic: "/my_isaac_joint_command"
```

Start the node with this parameter file. The Isaac Sim Bridge must use the same topic names.

---

## 9. Common Issues

### No `/joint_states` Messages

Check:

```shell
ros2 topic info /joint_states -v
ros2 topic echo /joint_states --once
```

If no messages are received, check whether the Isaac Sim ROS 2 Bridge is enabled and whether the ROS domain, RMW implementation, DDS discovery, and Docker network are configured consistently.

### MIT Is Rejected

This is the current design. Isaac Sim chassis nodes support only:

```text
VEL: chassis velocity control
NONE: empty mode; clears the currently retained command
```

When MIT is received, the node logs a warning and does not publish that control command.

### Meaning of `NONE`

`NONE` is an empty mode. It clears the currently retained control command and is not a new zero-velocity `VEL` command.

### No `/joint_command` Messages

Check:

```shell
ros2 topic info /joint_command -v
ros2 topic echo /joint_command --once
```

The Maver node also requires a valid `joint_state_topic` state. Check that the state topic contains all eight joint names and that the configured command topic matches the Isaac Sim Bridge subscription.

### ROS 2 Communication Fails Across Containers

Confirm that both sides use consistent:

```text
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
DDS discovery
Docker network
```

Do not use a configuration that restricts communication to the local loopback interface when running across containers.
