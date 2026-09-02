#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Forward Maver X4 controls to the Isaac Sim implicit PD controller."""

import os
import sys

import numpy as np

script_path = os.path.abspath(os.path.dirname(__file__))
if script_path not in sys.path:
    sys.path.append(script_path)

from maver_util import DataInterface
from maver_util.interface_base import MAVER_WHEEL_INDEX, MAVER_YAW_INDEX
from hex_util_msg.dataclass.dataclass_robo import HexDcRoboChsCtrlMode


CHS_DOF = 8
WHEEL_RADIUS = 0.0625
TRACK_WIDTH = 0.28
WHEEL_BASE = 0.424
BIAS = 0.02
# Wheel center positions, in the same order as yaw1..yaw4.
WHEEL_POS = np.array([
    [0.212, 0.14],
    [-0.212, 0.14],
    [-0.212, -0.14],
    [0.212, -0.14],
], dtype=np.float64)


class IsaacsimMaverX4:
    """Convert Maver X4 chassis controls into Bridge joint commands.

    The Isaac Sim Bridge uses yaw position targets for steering joints and
    wheel velocity targets for drive joints.  The node receives the current
    yaw positions from ``/joint_states`` before recalculating the command.
    """

    def __init__(self) -> None:
        """Initialize the Maver X4 transport and kinematic state."""
        self.__data_interface = DataInterface("hex_ros_isaacsim_maver_x4")

        rate_param = self.__data_interface.get_rate_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")

        # Store the latest state in the hex_ros_robot canonical order.
        self.__last_position = np.zeros(CHS_DOF)
        # Keep the newest accepted chassis command for repeated control cycles.
        self.__cur_ctrl = None
        # Limit the missing-Bridge warning to one message per node lifetime.
        self.__bridge_warning_sent = False
        # Limit the missing-control warning to one message per node lifetime.
        self.__control_warning_sent = False

    @staticmethod
    def __array(values, default: float = 0.0) -> np.ndarray:
        """Return an eight-joint array or a uniform default.

        Args:
            values: Input sequence to validate.
            default: Value used when the input does not contain eight items.

        Returns:
            A floating-point array with exactly eight elements.

        """
        array = np.asarray(values, dtype=np.float64)
        if array.size == CHS_DOF:
            return array
        return np.full(CHS_DOF, default, dtype=np.float64)

    @staticmethod
    def __wrap(angle: np.ndarray) -> np.ndarray:
        """Wrap angles to the half-open interval ``[-pi, pi)``.

        Args:
            angle: Angles in radians.

        Returns:
            Wrapped angles in radians.

        """
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def __calc_ik(
        self, twist: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Calculate canonical steering-angle and wheel-speed targets.

        This method uses analytical geometric inverse kinematics.  For each
        wheel, it first calculates the velocity of the wheel contact point
        from the chassis twist.  The velocity direction determines the
        steering angle, while its magnitude divided by the wheel radius
        determines the wheel angular velocity.

        Isaac Sim uses a position target for each steering actuator and a
        velocity target for each drive actuator.  This differs from MuJoCo,
        where the Jacobian output is treated as a generalized velocity target
        and tracked through a torque loop.  The geometric solution is used
        here because it directly produces the two target types expected by
        Isaac Sim.

        Args:
            twist: Chassis velocity ``[vx, vy, omega_z]`` in the body frame.

        Returns:
            A tuple containing canonical velocity and position targets.
            Wheel velocities are stored at indices ``[0, 2, 4, 6]`` and
            steering positions are stored at indices ``[1, 3, 5, 7]``.

        """
        vx, vy, omega = twist
        omega *=  1.0 / 0.86

        # Combine chassis translation with the rotational velocity at each
        # wheel location to obtain the required contact-point velocity.
        point_vel = np.column_stack((
            vx - omega * WHEEL_POS[:, 1],
            vy + omega * WHEEL_POS[:, 0],
        ))

        # The contact-point velocity direction is the required wheel heading.
        target_yaw = np.arctan2(point_vel[:, 1], point_vel[:, 0])

        # Convert linear contact-point speed to wheel angular speed.
        wheel_speed = np.linalg.norm(point_vel, axis=1) / WHEEL_RADIUS

        # Reverse the wheel instead of rotating the steering joint by more
        # than 90 degrees; both choices produce the same contact-point motion.
        current_yaw = self.__last_position[MAVER_YAW_INDEX]
        delta = self.__wrap(target_yaw - current_yaw)
        reverse = np.abs(delta) > (0.5 * np.pi)
        target_yaw = np.where(
            reverse,
            self.__wrap(target_yaw + np.pi),
            target_yaw,
        )
        wheel_speed = np.where(reverse, -wheel_speed, wheel_speed)

        velocity = np.zeros(CHS_DOF, dtype=np.float64)
        velocity[MAVER_WHEEL_INDEX] = wheel_speed

        position = np.zeros(CHS_DOF, dtype=np.float64)
        position[MAVER_YAW_INDEX] = target_yaw

        # Keep IK output in robot ROS order; the interface maps it to the
        # current Isaac Sim Bridge order before publishing.
        return velocity, position

    def __apply_chs_ctrl(self, ctrl):
        """Convert one chassis command into Isaac Sim joint targets.

        Args:
            ctrl: Chassis control message in the shared dataclass format.

        Returns:
            A tuple of velocity, effort, and position arrays, or ``None`` for
            an unsupported control mode.

        """
        chs_ctrl = ctrl.chs_ctrl
        if chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.VEL:
            # VEL mode derives actuator targets from the requested body twist.
            twist = np.array([
                chs_ctrl.vel.linear.x,
                chs_ctrl.vel.linear.y,
                chs_ctrl.vel.angular.z], dtype=np.float64)
            velocity, position = self.__calc_ik(twist)
            effort = self.__array(chs_ctrl.jnt.eff)
        elif chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.MIT:
            self.__data_interface.logw(
                "Maver MIT control is not supported; use VEL mode")
            return None
        else:
            return None

        # Arrays remain canonical; the interface maps them to Bridge order.
        return velocity, effort, position

    def run(self) -> None:
        """Run the fixed-rate control and command-forwarding loop.

        The newest chassis command is retained between loop iterations.  The
        newest Bridge joint state supplies steering angles for the next IK
        calculation.
        """
        while self.__data_interface.ok():
            ctrl = self.__data_interface.get_chs_ctrl(latest=True)
            if ctrl is not None:
                # NONE clears the retained command; other modes replace it.
                self.__data_interface.logd(
                    f"received chassis control: {ctrl.chs_ctrl.ctrl_mode}")
                if ctrl.chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.NONE:
                    self.__cur_ctrl = None
                else:
                    self.__cur_ctrl = ctrl

            # The IK depends on the actual steering angle.  Unlike MuJoCo,
            # Isaac Sim sends this state through the ROS bridge.
            joint_names = self.__data_interface.get_joint_names()
            if not joint_names:
                if not self.__bridge_warning_sent:
                    self.__data_interface.logw(
                        "ROS bridge joint_states not received; "
                        "command forwarding is disabled")
                    self.__bridge_warning_sent = True
                self.__data_interface.sleep()
                continue

            if (not self.__data_interface.has_valid_joint_state()
                    or len(joint_names) != CHS_DOF):
                self.__data_interface.sleep()
                continue

            # The interface has already mapped the latest Bridge frame into
            # canonical order before the control loop reads this snapshot.
            self.__last_position = self.__data_interface.get_joint_positions()

            if self.__cur_ctrl is not None:
                command = self.__apply_chs_ctrl(self.__cur_ctrl)
                if command is not None:
                    self.__data_interface.pub_joint_command(
                        joint_names, *command)
                    self.__data_interface.logd("published joint command")
                else:
                    self.__data_interface.logw(
                        "X4 control was rejected; no command published")
            elif not self.__control_warning_sent:
                self.__data_interface.logw(
                    "X4 chs_ctrl not received; command forwarding is idle")
                self.__control_warning_sent = True
            self.__data_interface.sleep()

    def shutdown(self) -> None:
        """Stop the ROS interface and release its resources."""
        self.__data_interface.shutdown()


def main() -> None:
    """Create and run the Maver X4 Bridge forwarder."""
    robot_maver_x4 = IsaacsimMaverX4()
    try:
        robot_maver_x4.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_maver_x4.shutdown()


if __name__ == "__main__":
    main()
