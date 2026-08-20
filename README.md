# hex_ros_robot_chassis
[中文](README_CN.md) | **English**

## Table of Contents

- [1. Package Overview](#1-package-overview)
- [2. Package Architecture](#2-package-architecture)
- [3. Topic Interface](#3-topic-interface)
- [4. Control Modes](#4-control-modes)
- [5. Parameter Reference](#5-parameter-reference)
- [6. Dependencies](#6-dependencies)
- [7. Quick Start](#7-quick-start)

---

## 1. Package Overview

This is the **ROS Driver Package** for **HEXFELLOW** chassis.

- **Trigger A3 LR1** — a three-wheel omnidirectional chassis with three omni wheels, enabling 3-DoF planar motion (forward/backward, left/right, rotation).
- **Trigger A3 H1** — a 3-motor variant sharing the same chassis as **Trigger A3 LR1**; the difference is the motors: H1 supports true **MIT impedance control**, whereas LR1's `MIT` semantics is "target speed + max current limit".
- **Maver** — a four-wheel steering chassis with 8 motors (4 steering joints `joint_yaw1` ~ `joint_yaw4` + 4 drive joints `joint_wheel1` ~ `joint_wheel4`), supporting two hardware variants: **X4H1** (`robot_type=30`) and **L4H1** (`robot_type=31`).

> The three drive joints of the A3 chassis (LR1 / H1) are uniformly named `joint_1` ~ `joint_3`, consistent with the URDF of [hex_ros_urdf_trigger_a](https://github.com/hexfellow/hex_ros_urdf_trigger_a) and the [hex_ros_sim_trigger_a](https://github.com/hexfellow/hex_ros_sim_trigger_a) simulation.

The package connects to the chassis controller via WebSocket, forwards ROS control commands to the hardware, and publishes real-time chassis state, odometry, joint states, and TF transforms.
Supports **ROS 1** and **ROS 2**.

---

## 2. Package Architecture

```
hex_ros_robot_chassis/
├── config/                              # Parameter configuration
│   ├── ros1/
│   │   ├── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 1 params
│   │   ├── trigger_a3_h1_params.yaml    #   A3 H1 ROS 1 params
│   │   ├── maver_params.yaml            #   Maver ROS 1 params
│   │   └── display_maver_x4.rviz        #   Maver X4 ROS 1 rviz config
│   └── ros2/
│       ├── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 2 params
│       ├── trigger_a3_h1_params.yaml    #   A3 H1 ROS 2 params
│       ├── maver_params.yaml            #   Maver ROS 2 params
│       └── display_maver_x4.rviz        #   Maver X4 ROS 2 rviz config
├── launch/                              # Launch files
│   ├── ros1/
│   │   ├── trigger_a3_lr1.launch        #   A3 LR1 ROS 1 launch
│   │   ├── trigger_a3_h1.launch         #   A3 H1 ROS 1 launch (with rviz, for demo include)
│   │   └── maver.launch                 #   Maver ROS 1 launch (with rviz)
│   └── ros2/
│       ├── trigger_a3_lr1.launch.py     #   A3 LR1 ROS 2 launch
│       ├── trigger_a3_h1.launch.py      #   A3 H1 ROS 2 launch (with rviz, for demo include)
│       └── maver.launch.py              #   Maver ROS 2 launch (with rviz)
├── hex_ros_robot_chassis/               # Core source
│   ├── robot_trigger_a3_lr1.py          #   A3 LR1 main node (control loop + ROS interface)
│   ├── robot_trigger_a3_h1.py           #   A3 H1 main node (control loop + ROS interface, MIT impedance)
│   ├── robot_maver.py                   #   Maver main node (control loop + ROS interface)
│   ├── utility/                         #   DataInterface shared by Trigger A3 LR1 / H1
│   │   ├── __init__.py                  #     Auto-selects ROS1/ROS2 impl via ROS_VERSION
│   │   ├── interface_base.py            #     Abstract base class ChassisInterfaceBase
│   │   ├── ros1_interface.py            #     ROS 1 DataInterface implementation
│   │   └── ros2_interface.py            #     ROS 2 DataInterface implementation
│   └── maver_util/                      #   Maver DataInterface (same structure as utility/)
│       ├── __init__.py                  #     Auto-selects ROS1/ROS2 impl via ROS_VERSION
│       ├── interface_base.py            #     Abstract base class ChassisInterfaceBase
│       ├── ros1_interface.py            #     ROS 1 DataInterface implementation
│       └── ros2_interface.py            #     ROS 2 DataInterface implementation
├── resource/                            # ament resource files
├── setup.py                             # Python packaging (3 entry_points)
├── setup.cfg                            # Python packaging config
├── package.xml                          # ROS package manifest (dual-system conditional deps)
└── README.md                            # English docs
```

### Interface Layer

`utility/` (Trigger A3 LR1 / H1) and `maver_util/` (Maver) each provide a unified `DataInterface` that automatically selects the ROS 1 or ROS 2 implementation based on the `ROS_VERSION` environment variable:

| Implementation | File | Compatible Versions |
|----------------|------|---------------------|
| `ros1_interface.DataInterface` | `ros1_interface.py` | ROS 1 (Noetic) |
| `ros2_interface.DataInterface` | `ros2_interface.py` | ROS 2 (Humble / Foxy) |

> `maver_util/` has the same structure as `utility/` and is the Maver-specific interface layer (it additionally reads the `robot_type` parameter). Both A3 LR1 and A3 H1 reuse `utility/` (the A3 H1 model is fixed, so no `robot_type` is needed).

---

## 3. Topic Interface

### Trigger A3 LR1

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | Chassis control command (VEL / MIT modes) |
| Publish | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | Chassis state feedback (3-wheel position, velocity, torque) |
| Publish | `odom` | `nav_msgs/(msg/)Odometry` | Odometry (position x, y, yaw + velocity vx, vy, omega) |
| Publish | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 wheel joint states (`joint_1` ~ `joint_3`) |
| Publish | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link transform |

### Trigger A3 H1

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | Chassis control command (VEL / MIT modes) |
| Publish | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | Chassis state feedback (3-motor position, velocity, torque) |
| Publish | `odom` | `nav_msgs/(msg/)Odometry` | Odometry (position x, y, yaw + velocity vx, vy, omega) |
| Publish | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 joint states (`joint_1` ~ `joint_3`) |
| Publish | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link transform |

> A3 LR1 and A3 H1 publish the same topics and joint names (`joint_1` ~ `joint_3`); they differ only in the drive motors and control semantics (see "4. Control Modes").

### Maver

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | Chassis control command (VEL / MIT modes) |
| Publish | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | Chassis state feedback (8-motor position, velocity, torque) |
| Publish | `odom` | `nav_msgs/(msg/)Odometry` | Odometry (position x, y, yaw + velocity vx, vy, omega) |
| Publish | `joint_states` | `sensor_msgs/(msg/)JointState` | 8 joint states (`joint_wheel1`, `joint_yaw1`, ..., `joint_wheel4`, `joint_yaw4`) |
| Publish | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link transform |

> **Joint order**: The 8 joints of Maver X4 (array indices 0~7) are ordered `joint_wheel1, joint_yaw1, joint_wheel2, joint_yaw2, joint_wheel3, joint_yaw3, joint_wheel4, joint_yaw4`; indices 0, 2, 4, 6 are drive (wheel) joints and indices 1, 3, 5, 7 are steering (yaw) joints. Both the `jnt` array of `chs_ctrl` and `joint_states` must follow this order.

> Message type definitions: [hex_ros_msgs](https://github.com/hexfellow/hex_ros_msgs)

> ROS timestamps are provided by default; for hardware timestamps, use `chs_state.jnt.header.stamp`.

---

## 4. Control Modes

The chassis supports two control modes, selected via `chs_ctrl.ctrl_mode`:

| Mode | Value | Description |
|------|-------|-------------|
| `VEL` | `2` | Velocity mode: send (vx, vy, omega) 3-DoF velocity commands |
| `MIT` | `1` | Torque mode: semantics vary by motor driver (see the per-model notes below) |
| `NONE` | `0` | No operation, no control executed |

> **Trigger A3 LR1**: MIT mode sends **target speed (rad/s) + max current limit (A)**, mapped to `set_chs_per_motor_spd_cmd` — not true impedance control.

> **Trigger A3 H1**: MIT mode is **true impedance control** via `set_chs_mit_cmd`, sending position/velocity/stiffness/damping; the current firmware forces `kp=0`.

> **Maver**: the current firmware forces `kp=0` in MIT mode.

### MIT Mode Usage Warning

> **MIT Mode Usage Warning**: Do not use MIT mode unless you know what MIT mode is;

> Improper use may cause violent chassis motion or even equipment damage.

> Ensure operation in a safe area and be ready for emergency stop at any time.

---

## 5. Parameter Reference

### Trigger A3 LR1

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ctrl_rate` | 1000.0 | Main control loop rate [Hz] |
| `rate_state` | 500.0 | State publish rate (decimated from ctrl_rate) [Hz] |
| `robot_host` | 192.168.1.100 | Chassis controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `state_buffer_size` | 200 | Driver state buffer size |
| `enable_kcp` | `true` | Enable KCP transport protocol |

### Trigger A3 H1

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ctrl_rate` | 1000.0 | Main control loop rate [Hz] |
| `rate_state` | 500.0 | State publish rate (decimated from ctrl_rate) [Hz] |
| `robot_host` | 192.168.1.100 | Chassis controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `state_buffer_size` | 200 | Driver state buffer size |
| `sens_ts` | `true` | Use hardware sensor timestamps |
| `enable_kcp` | `true` | Enable KCP transport protocol |

> A3 H1 has the same parameters as LR1 (no `robot_type`; the model is fixed to H1).

### Maver

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ctrl_rate` | 1000.0 | Main control loop rate [Hz] |
| `rate_state` | 500.0 | State publish rate [Hz] |
| `robot_host` | 192.168.1.100 | Chassis controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `state_buffer_size` | 200 | Driver state buffer size |
| `sens_ts` | `true` | Use hardware sensor timestamps |
| `enable_kcp` | `true` | Enable KCP transport protocol |
| `robot_type` | 30 | Chassis model: 30=X4H1, 31=L4H1 |

> `rate_state` (state publish rate) defaults to 500.0. Note: the value in the ROS 2 config `config/ros2/maver_params.yaml` is 1000.0; 500.0 is authoritative.

---

## 6. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0a4'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

> A3 chassis (LR1 / H1) rviz visualization (launch `rviz:=true`) requires an additional URDF package:

```shell
git clone https://github.com/hexfellow/hex_ros_urdf_trigger_a.git
```

> Maver X4 rviz visualization (launch `rviz:=true`) requires an additional URDF package:

```shell
git clone https://github.com/hexfellow/hex_ros_urdf_maver_x4.git
```

---

## 7. Quick Start

### 1. Create a workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone the packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
git clone https://github.com/hexfellow/hex_ros_urdf_trigger_a.git    # A3 chassis (LR1 / H1)
git clone https://github.com/hexfellow/hex_ros_urdf_maver_x4.git     # Maver X4
```

### 3. Build the packages

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash --extend
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash --extend
```

### 4. Use the package

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

Optional arguments for `trigger_a3_h1.launch.py` / `maver.launch.py`:
- `rviz:=true/false`: whether to launch rviz visualization (default `true`; A3 requires `hex_ros_urdf_trigger_a`, Maver requires `hex_ros_urdf_maver_x4`)

**Selecting the Maver model:**

Maver has two models, **X4H1** and **L4H1**, selected via the `robot_type` parameter (default `30` = X4H1):

| Model | `robot_type` |
|-------|--------------|
| X4H1 | `30` |
| L4H1 | `31` |

The model is selected by editing the parameter config file for the corresponding ROS version, then launching:

- **ROS 2**: edit `robot_type` in `config/ros2/maver_params.yaml`, then launch `maver.launch.py`
- **ROS 1**: edit `robot_type` in `config/ros1/maver_params.yaml`, then launch `maver.launch`

> Replace `robot_host` and `robot_port` with the actual chassis controller IP and port.

### 5. Control the chassis

#### Quick Test with Trigger A3 LR1

Use `ros2 topic pub` to quickly send control commands:

```bash
# VEL mode — rotate 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# VEL mode — stop
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# Target speed + max current limit mode — 3 motors at 0.3, current limit 2.0
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# Target speed + max current limit mode — stop
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

```

Use `rostopic pub` for ROS 1:

```bash
# VEL mode — rotate 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# VEL mode — stop
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Target speed + max current limit mode — 3 motors at 0.3, current limit 2.0
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Target speed + max current limit mode — stop
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> MIT mode is not currently supported by the **Trigger A3 lr**; when used, the driver will send target speed (rad/s) + max current limit (A) to the **Trigger A3 lr** device.

#### Quick Test with Trigger A3 H1

Use `ros2 topic pub` to quickly send control commands (the 3 motors are ordered `joint_1` ~ `joint_3`):

```bash
# VEL mode — rotate 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# MIT mode — damped motion (target speed 0.5 rad/s, damping 3.0, kp forced to 0 by current firmware)
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# MIT mode — release (all zero, no output torque)
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0], vel: [0, 0, 0], eff: [0, 0, 0], kp: [0, 0, 0], kd: [0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

Use `rostopic pub` (ROS 1) to send control commands:

```bash
# VEL mode — rotate 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# MIT mode — damped motion (target speed 0.5 rad/s, damping 3.0, kp forced to 0 by current firmware)
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# MIT mode — release (all zero, no output torque)
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0], vel: [0, 0, 0], eff: [0, 0, 0], kp: [0, 0, 0], kd: [0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> A3 H1's MIT is true impedance control (`set_chs_mit_cmd`); the fields match Maver: `pos` target position, `vel` target velocity, `kp` position stiffness, `kd` damping; the current firmware forces `kp=0`.

#### Quick Test with Maver

Use `ros2 topic pub` to quickly send control commands (the 8 motors are ordered `joint_wheel1, joint_yaw1, ..., joint_wheel4, joint_yaw4`):

```bash
# VEL mode — forward 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# VEL mode — rotate in place 0.5 rad/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# MIT mode — damped motion (target speed 0.5 rad/s, damping 3.0, kp forced to 0 by current firmware)
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# MIT mode — release (all zero, no output torque)
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0, 0, 0, 0, 0, 0], vel: [0, 0, 0, 0, 0, 0, 0, 0], eff: [0, 0, 0, 0, 0, 0, 0, 0], kp: [0, 0, 0, 0, 0, 0, 0, 0], kd: [0, 0, 0, 0, 0, 0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

Use `rostopic pub` (ROS 1) to send control commands:

```bash
# VEL mode — rotate 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}}}"

# MIT mode — damped motion (target speed 0.5 rad/s, damping 3.0, kp forced to 0 by current firmware)
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], eff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# MIT mode — release (all zero, no output torque)
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [0, 0, 0, 0, 0, 0, 0, 0], vel: [0, 0, 0, 0, 0, 0, 0, 0], eff: [0, 0, 0, 0, 0, 0, 0, 0], kp: [0, 0, 0, 0, 0, 0, 0, 0], kd: [0, 0, 0, 0, 0, 0, 0, 0], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

MIT mode field description:

| `chs_ctrl.jnt` field | Meaning | Unit |
|----------------------|---------|------|
| `jnt.pos[0..7]` | Target position | rad |
| `jnt.vel[0..7]` | Target velocity | rad/s |
| `jnt.kp[0..7]` | Position stiffness | Nm/rad |
| `jnt.kd[0..7]` | Damping | Nm/(rad/s) |

> `--once` publishes a single message that only takes effect for one control cycle; use `--rate` to publish periodically for continuous motion.
> `robot_type` model selection: see "4. Use the package" above (30=X4H1 / 31=L4H1).
> The array order of the 8 motors is `joint_wheel1, joint_yaw1, joint_wheel2, joint_yaw2, joint_wheel3, joint_yaw3, joint_wheel4, joint_yaw4` (indices 0, 2, 4, 6 = wheel drive, 1, 3, 5, 7 = yaw steering).
