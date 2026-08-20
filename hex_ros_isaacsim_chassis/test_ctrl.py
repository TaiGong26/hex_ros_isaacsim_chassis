#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Test controller – cycles chassis between MIT position presets and VEL
velocity commands, and reports the received chs_state frequency."""

import threading
import time

import numpy as np

import rclpy
import rclpy.node

from hex_ros_msgs.msg import HexRosRoboChsCtrlStamped, HexRosRoboChsStateStamped
from builtin_interfaces.msg import Time

# ctrl_mode values mirror hex_util_msg.HexDcRoboChsCtrlMode
CHS_CTRL_MODE_NONE = 0
CHS_CTRL_MODE_MIT = 1
CHS_CTRL_MODE_VEL = 2

CYCLE_PERIOD = 5.0

# MIT position presets (per joint, in canonical joint order)
MIT_POS_PRESETS = [
    [0.0] * 8,
    [0.5, 0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0],
    [-0.5, -0.2, 0.3, -0.1, 0.0, 0.0, 0.0, 0.0],
]
MIT_KP = 50.0
MIT_KD = 10.0

# VEL commands: (vx, vy, omega)
VEL_PRESETS = [
    (0.0, 0.0, 0.0),
    (0.2, 0.0, 0.0),
    (0.0, 0.2, 0.0),
    (0.0, 0.0, 0.5),
]
VEL_KD = 10.0


class ChassisTestNode:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.node.Node("test_ctrl")
        self.logger = self.node.get_logger()

        self.node.declare_parameter("control_mode", "vel")   # 'mit' | 'vel'
        self.node.declare_parameter("joint_count", 8)
        self.node.declare_parameter("publish_rate", 100.0)
        self.node.declare_parameter("chs_ctrl_topic", "chs_ctrl")

        self.control_mode = self.node.get_parameter("control_mode").value
        self.joint_count = int(self.node.get_parameter("joint_count").value)
        self.publish_rate = float(
            self.node.get_parameter("publish_rate").value)

        self.ctrl_pub = self.node.create_publisher(
            HexRosRoboChsCtrlStamped,
            self.node.get_parameter("chs_ctrl_topic").value,
            10,
        )
        self.sub = self.node.create_subscription(
            HexRosRoboChsStateStamped,
            "chs_state",
            self.state_callback,
            10,
        )
        self.state_counter = 0

        self.rate = self.node.create_rate(self.publish_rate)
        self.cycle_decimation = max(
            1, int(round(CYCLE_PERIOD * self.publish_rate)))
        self.logger.info(f"[test_ctrl] mode={self.control_mode} "
                         f"joint_count={self.joint_count}")

        # dedicated spin thread: rclpy.spin_once(timeout_sec=0.0) in a tight
        # loop blocks the executor in Humble, so callbacks (chs_state) must be
        # driven from a separate thread while run() stays on the publish loop.
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def state_callback(self, _msg):
        self.state_counter += 1

    def _build_ctrl(self, cycle_index: int):
        msg = HexRosRoboChsCtrlStamped()
        now = self.node.get_clock().now()
        msg.header.stamp = Time(sec=int(now.seconds_nanoseconds()[0]),
                                nanosec=int(now.seconds_nanoseconds()[1]))
        msg.header.frame_id = "test_ctrl"

        n = self.joint_count
        z = [0.0] * n
        kd = [MIT_KD if self.control_mode == 'mit' else VEL_KD] * n
        idx = cycle_index % len(MIT_POS_PRESETS)

        if self.control_mode == 'mit':
            msg.chs_ctrl.ctrl_mode = CHS_CTRL_MODE_MIT
            # preset is length 8; truncate/pad to the joint_count (3 for trigger)
            pos = (MIT_POS_PRESETS[idx] + z)[:n]
            msg.chs_ctrl.jnt.pos = pos
            msg.chs_ctrl.jnt.vel = z
            msg.chs_ctrl.jnt.eff = z
            msg.chs_ctrl.jnt.kp = [MIT_KP] * n
            msg.chs_ctrl.jnt.kd = kd
            msg.chs_ctrl.jnt.lim_vel = z
            msg.chs_ctrl.jnt.lim_acc = z
        else:
            msg.chs_ctrl.ctrl_mode = CHS_CTRL_MODE_VEL
            vx, vy, omega = VEL_PRESETS[idx]
            msg.chs_ctrl.jnt.pos = z
            msg.chs_ctrl.jnt.vel = z
            msg.chs_ctrl.jnt.eff = z
            msg.chs_ctrl.jnt.kp = [0.0] * n
            msg.chs_ctrl.jnt.kd = kd
            msg.chs_ctrl.jnt.lim_vel = z
            msg.chs_ctrl.jnt.lim_acc = z
            msg.chs_ctrl.vel.linear.x = vx
            msg.chs_ctrl.vel.linear.y = vy
            msg.chs_ctrl.vel.angular.z = omega
        return msg

    def run(self):
        loop_counter = 0
        last_report_time = time.monotonic()
        while rclpy.ok():
            if loop_counter % self.cycle_decimation == 0:
                cycle_index = loop_counter // self.cycle_decimation
                msg = self._build_ctrl(cycle_index)
                self.ctrl_pub.publish(msg)

            now = time.monotonic()
            if now - last_report_time >= 1.0:
                elapsed = now - last_report_time
                self.logger.info(
                    f"chs_state receive frequency: "
                    f"{self.state_counter / elapsed:.1f} Hz")
                self.state_counter = 0
                last_report_time = now

            loop_counter += 1
            self.rate.sleep()

    def shutdown(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    node = ChassisTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[test node] error: {e}")
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
