"""ROS 2 interface for the Isaac Sim X4 forwarder.

Copyright 2026 Dong Zhaorui. All rights reserved.
Author: Dong Zhaorui 847235539@qq.com
Date: 2026-08-31
"""

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
    """Provide ROS transport while leaving control logic to the robot node."""

    def __init__(self, name: str = "unknown") -> None:
        rclpy.init()
        self.__node = rclpy.node.Node(
            name, automatically_declare_parameters_from_overrides=True)
        self._logger = self.__node.get_logger()
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
        """Sleep according to the ROS clock."""
        self.__rate.sleep()

    def ok(self) -> bool:
        """Return whether ROS is active."""
        return rclpy.ok()

    def shutdown(self) -> None:
        """Destroy the node and stop the ROS context."""
        if self._shutting_down:
            return
        self._shutting_down = True
        self.__node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.__spin_thread.join(timeout=1.0)

    def __spin(self) -> None:
        """Spin the ROS node until shutdown."""
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
        """Publish Isaac actuator targets and feed-forward effort."""
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.velocity = np.asarray(velocity, dtype=np.float64).tolist()
        msg.effort = np.asarray(effort, dtype=np.float64).tolist()
        if position is not None:
            msg.position = np.asarray(position, dtype=np.float64).tolist()
        self.__command_pub.publish(msg)

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        """Convert a ROS joint command to the shared dataclass."""
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64))

    def __ctrl_callback(self, msg: HexRosRoboChsCtrlStamped) -> None:
        """Queue an incoming chassis command."""
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
        """Capture bridge names and update the latest positions."""
        self.set_joint_state(msg.name, msg.position)
