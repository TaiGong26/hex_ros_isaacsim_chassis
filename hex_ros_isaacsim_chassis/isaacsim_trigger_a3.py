#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Forward Trigger A3 controls to the Isaac Sim implicit PD controller."""

import os
import sys

import numpy as np

script_path = os.path.abspath(os.path.dirname(__file__))
if script_path not in sys.path:
    sys.path.append(script_path)

from utility import DataInterface
from hex_util_msg.dataclass.dataclass_robo import HexDcRoboChsCtrlMode


CHS_DOF = 3
JOINT_NAME = [f"joint_{i}" for i in range(1, CHS_DOF + 1)]


class IsaacsimTriggerA3:
    """Convert Trigger A3 controls into Bridge joint commands."""

    def __init__(self) -> None:
        """Initialize the Trigger A3 transport and kinematic state."""
        self.__data_interface = DataInterface("hex_ros_isaacsim_trigger_a3")

        rate_param = self.__data_interface.get_rate_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")

        self.__chs_params = {
            "wheel_radius": 0.102,
            "wheel_distance": 0.3252,
        }
        beta = np.array([np.pi / 3,np.pi , -np.pi / 3])
        s, c = np.sin(beta), np.cos(beta)
        self.__jac_inv = -1.0 / self.__chs_params["wheel_radius"] * np.array([
            [-s[0], c[0], self.__chs_params["wheel_distance"]],
            [-s[1], c[1], self.__chs_params["wheel_distance"]],
            [-s[2], c[2], self.__chs_params["wheel_distance"]],
        ])
        
        # Keep the newest accepted chassis command for repeated control cycles.
        self.__cur_ctrl = None
        # Limit the missing-control warning to one message per node lifetime.
        self.__control_warning_sent = False

    @staticmethod
    def __array(values, default: float = 0.0) -> np.ndarray:
        """Return a three-joint array or a uniform default.

        Args:
            values: Input sequence to validate.
            default: Value used when the input does not contain three items.

        Returns:
            A floating-point array with exactly three elements.

        """
        array = np.asarray(values, dtype=np.float64)
        if array.size == CHS_DOF:
            return array
        return np.full(CHS_DOF, default, dtype=np.float64)

    def __apply_chs_ctrl(self, ctrl):
        """Convert one A3 chassis command into joint targets.

        Args:
            ctrl: Chassis control message in the shared dataclass format.

        Returns:
            A tuple of velocity and effort arrays, or ``None`` for an
            unsupported control mode.

        """
        chs_ctrl = ctrl.chs_ctrl
        if chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.VEL:
            # VEL mode maps the requested body twist through the fixed A3 IK.
            twist = np.array([
                chs_ctrl.vel.linear.x,
                chs_ctrl.vel.linear.y,
                chs_ctrl.vel.angular.z])
            velocity = self.__jac_inv @ twist
            effort = self.__array(chs_ctrl.jnt.eff)
        elif chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.MIT:
            self.__data_interface.logw(
                "Trigger A3 MIT control is not supported; use VEL mode")
            return None
        else:
            return None

        return velocity, effort

    def run(self) -> None:
        """Run the fixed-rate A3 command-forwarding loop."""
        while self.__data_interface.ok():
            ctrl = self.__data_interface.get_chs_ctrl(latest=True)
            if ctrl is not None:
                # NONE clears the retained command; other modes replace it.
                if ctrl.chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.NONE:
                    self.__cur_ctrl = None
                else:
                    self.__cur_ctrl = ctrl

            if self.__cur_ctrl is not None:
                command = self.__apply_chs_ctrl(self.__cur_ctrl)
                if command is not None:
                    self.__data_interface.pub_joint_command(JOINT_NAME, *command)
                else:
                    self.__data_interface.logw(
                        "A3 control was rejected; no command published")
            elif not self.__control_warning_sent:
                self.__data_interface.logw(
                    "A3 chs_ctrl not received; command forwarding is idle")
                self.__control_warning_sent = True
            self.__data_interface.sleep()

    def shutdown(self) -> None:
        """Stop the ROS interface and release its resources."""
        self.__data_interface.shutdown()


def main() -> None:
    """Create and run the Trigger A3 Bridge forwarder."""
    robot_trigger_a3 = IsaacsimTriggerA3()
    try:
        robot_trigger_a3.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_trigger_a3.shutdown()


if __name__ == "__main__":
    main()
