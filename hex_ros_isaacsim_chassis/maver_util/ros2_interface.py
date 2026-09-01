#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Provide the ROS 2 interface for the Isaac Sim Maver X4 bridge."""

import threading

import numpy as np
import rclpy
import rclpy.node
from sensor_msgs.msg import JointState
from hex_ros_msgs.msg import HexRosJnt, HexRosRoboChsCtrlStamped

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader, HexDcBaseJntFull, HexDcBaseTwist, HexDcBaseVector3,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl, HexDcRoboChsCtrlMode, HexDcRoboChsCtrlStamped,
)
from .interface_base import ChassisInterfaceBase


class DataInterface(ChassisInterfaceBase):
    """Provide ROS 2 transport services for the Maver X4 bridge.

    ROS callbacks run in a dedicated spin thread.  The chassis node reads
    control commands and cached joint positions from its control thread.
    """

    def __init__(self, name: str = "unknown") -> None:
        """Initialize ROS 2 publishers, subscribers, and the spin thread.

        Args:
            name: ROS 2 node name.

        """
        rclpy.init()
        self.__node = rclpy.node.Node(
            name, automatically_declare_parameters_from_overrides=True)
        self._logger = self.__node.get_logger()
        # Prevent repeated shutdown calls from destroying ROS resources twice.
        self._shutting_down = False
        super().__init__(name)

        self._rate_param["ros"] = float(
            self.__node.get_parameter("ctrl_rate").value)
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        self.__command_pub = self.__node.create_publisher(
            JointState,
            self.__node.get_parameter("joint_command_topic").value,
            10)
        self.__ctrl_sub = self.__node.create_subscription(
            HexRosRoboChsCtrlStamped,
            "chs_ctrl",
            self.__ctrl_callback,
            10)
        self.__joint_state_sub = self.__node.create_subscription(
            JointState,
            self.__node.get_parameter("joint_state_topic").value,
            self.__joint_state_callback,
            10)

        self.__spin_thread = threading.Thread(
            target=self.__spin, daemon=True)
        self.__spin_thread.start()

    def sleep(self) -> None:
        """Sleep for one configured control period."""
        self.__rate.sleep()

    def ok(self) -> bool:
        """Return whether the ROS 2 context is still active.

        Returns:
            ``True`` while the ROS 2 context is running.

        """
        return rclpy.ok()

    def shutdown(self) -> None:
        """Destroy ROS resources and stop the spin thread."""
        if self._shutting_down:
            return
        self._shutting_down = True
        self.__node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.__spin_thread.join(timeout=1.0)

    def __spin(self) -> None:
        """Process ROS callbacks until the context is shut down."""
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception:
            if not self._shutting_down:
                raise

    def logi(self, msg, *args, **kwargs) -> None:
        """Write an informational log."""
        self._logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs) -> None:
        """Write a warning log."""
        self._logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs) -> None:
        """Write an error log."""
        self._logger.error(msg, *args, **kwargs)

    def logd(self, msg, *args, **kwargs) -> None:
        """Write a debug log."""
        self._logger.debug(msg, *args, **kwargs)

    def pub_joint_command(self, names, velocity, effort, position=None) -> None:
        """Publish canonical joint targets in the current Bridge order.

        Args:
            names: Bridge joint names defining the output order.
            velocity: Eight canonical joint velocity targets.
            effort: Eight canonical effort feed-forward values.
            position: Optional eight canonical joint position targets.

        """
        bridge_velocity = self.to_bridge_order(velocity, names)
        bridge_effort = self.to_bridge_order(effort, names)
        bridge_position = (self.to_bridge_order(position, names)
                           if position is not None else None)
        if (bridge_velocity is None or bridge_effort is None
                or (position is not None and bridge_position is None)):
            self.logw("Maver command mapping is unavailable; command dropped")
            return

        # Publish arrays only after all three fields share one name mapping.
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.velocity = bridge_velocity.tolist()
        msg.effort = bridge_effort.tolist()
        if bridge_position is not None:
            msg.position = bridge_position.tolist()
        self.__command_pub.publish(msg)

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        """Convert a ROS joint command into the shared dataclass.

        Args:
            jnt: ROS joint command fields.

        Returns:
            A NumPy-backed shared joint-command structure.

        """
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64))

    def __ctrl_callback(self, msg: HexRosRoboChsCtrlStamped) -> None:
        """Convert and queue an incoming chassis command.

        Args:
            msg: ROS 2 chassis control message.

        """
        ctrl = msg.chs_ctrl
        self.logd(f"received chassis control: {ctrl.ctrl_mode}")
        self._chs_ctrl_deque.append(HexDcRoboChsCtrlStamped(
            header=HexDcBaseHeader(frame_id=msg.header.frame_id),
            chs_ctrl=HexDcRoboChsCtrl(
                ctrl_mode=HexDcRoboChsCtrlMode(int(ctrl.ctrl_mode)),
                jnt=self.__jnt_to_dc(ctrl.jnt),
                vel=HexDcBaseTwist(
                    linear=HexDcBaseVector3(
                        x=ctrl.vel.linear.x,
                        y=ctrl.vel.linear.y,
                        z=ctrl.vel.linear.z),
                    angular=HexDcBaseVector3(
                        x=ctrl.vel.angular.x,
                        y=ctrl.vel.angular.y,
                        z=ctrl.vel.angular.z)))))

    def __joint_state_callback(self, msg: JointState) -> None:
        """Cache the latest Bridge joint names and positions.

        Args:
            msg: Joint state message published by Isaac Sim.

        """
        self.set_joint_state(msg.name, msg.position)
