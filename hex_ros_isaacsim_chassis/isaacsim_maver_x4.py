"""Forward Maver X4 controls to the Isaac Sim implicit PD controller.

Copyright 2026 Dong Zhaorui. All rights reserved.
Author: Dong Zhaorui 847235539@qq.com
Date: 2026-08-31
"""

import os
import sys

import numpy as np

script_path = os.path.abspath(os.path.dirname(__file__))
if script_path not in sys.path:
    sys.path.append(script_path)

from maver_util import DataInterface
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
    """Convert X4 controls into fixed-rate bridge joint commands."""

    def __init__(self) -> None:
        self.__data_interface = DataInterface("hex_ros_isaacsim_maver_x4")

        rate_param = self.__data_interface.get_rate_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")

        self.__jac_inv = np.empty((CHS_DOF, 3), dtype=np.float64)
        self.__wheel_distance = 0.5 * np.sqrt(
            TRACK_WIDTH**2 + WHEEL_BASE**2)
        beta = np.arctan2(TRACK_WIDTH, WHEEL_BASE)
        self.__beta = np.array([
            beta, np.pi - beta, beta - np.pi, -beta])
        # Updated from Isaac Sim JointState.  The bridge currently places
        # yaw joints at indices 0..3 and wheel joints at indices 4..7.
        self.__last_position = np.zeros(CHS_DOF)
        self.__cur_ctrl = None
        self.__bridge_warning_sent = False
        self.__control_warning_sent = False

    @staticmethod
    def __array(values, default: float = 0.0) -> np.ndarray:
        """Return an eight-joint array or a uniform default."""
        array = np.asarray(values, dtype=np.float64)
        if array.size == CHS_DOF:
            return array
        return np.full(CHS_DOF, default, dtype=np.float64)

    @staticmethod
    def __wrap(angle: np.ndarray) -> np.ndarray:
        """Wrap angles to [-pi, pi)."""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def __calc_ik(self, twist: np.ndarray):
        """Return Isaac position/velocity targets for a 4WIS chassis.

        The MuJoCo Jacobian produces yaw *rates*.  Isaac's implicit steering
        actuator, however, expects a yaw *position* target.  Sending the
        Jacobian yaw rates as JointState.velocity therefore only nudges the
        steering joints and cannot produce a stable pure rotation.
        """
        vx, vy, omega = twist
        point_vel = np.column_stack((
            vx - omega * WHEEL_POS[:, 1],
            vy + omega * WHEEL_POS[:, 0],
        ))
        target_yaw = np.arctan2(point_vel[:, 1], point_vel[:, 0])
        wheel_speed = np.linalg.norm(point_vel, axis=1) / WHEEL_RADIUS

        # Avoid unnecessary 180-degree steering moves.  Reversing the wheel
        # is kinematically equivalent and keeps the target near current yaw.
        current_yaw = self.__last_position[:4]
        delta = self.__wrap(target_yaw - current_yaw)
        reverse = np.abs(delta) > (0.5 * np.pi)
        target_yaw = np.where(reverse,
                              self.__wrap(target_yaw + np.pi), target_yaw)
        wheel_speed = np.where(reverse, -wheel_speed, wheel_speed)

        velocity = np.zeros(CHS_DOF, dtype=np.float64)
        velocity[4:8] = wheel_speed
        position = np.zeros(CHS_DOF, dtype=np.float64)
        position[:4] = target_yaw
        return velocity, position

    def __apply_chs_ctrl(self, ctrl):
        """Apply X4 kinematics or directly forward an MIT command."""
        chs_ctrl = ctrl.chs_ctrl
        if chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.VEL:
            twist = np.array([
                chs_ctrl.vel.linear.x,
                chs_ctrl.vel.linear.y,
                chs_ctrl.vel.angular.z], dtype=np.float64)
            velocity, position = self.__calc_ik(twist)
            effort = self.__array(chs_ctrl.jnt.eff)
        elif chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.MIT:
            velocity = self.__array(chs_ctrl.jnt.vel)
            effort = self.__array(chs_ctrl.jnt.eff)
            position = self.__array(chs_ctrl.jnt.pos)
        else:
            return None

        return velocity, effort, position

    def run(self) -> None:
        """Run the fixed-rate command-forwarding loop."""
        while self.__data_interface.ok():
            ctrl = self.__data_interface.get_chs_ctrl(latest=True)
            if ctrl is not None:
                self.__data_interface.logd(
                    f"received chassis control: {ctrl.chs_ctrl.ctrl_mode}")
                if ctrl.chs_ctrl.ctrl_mode == HexDcRoboChsCtrlMode.NONE:
                    self.__cur_ctrl = None
                else:
                    self.__cur_ctrl = ctrl

            # The IK depends on the actual steering angle.  Unlike MuJoCo,
            # Isaac Sim sends this state through the ROS bridge.
            joint_names = self.__data_interface.get_joint_names()
            bridge_position = self.__data_interface.get_joint_positions()
            if len(joint_names) == CHS_DOF:
                self.__last_position = bridge_position
            if not joint_names:
                if not self.__bridge_warning_sent:
                    self.__data_interface.logw(
                        "ROS bridge joint_states not received; "
                        "command forwarding is disabled")
                    self.__bridge_warning_sent = True
                self.__data_interface.sleep()
                continue

            if len(joint_names) != CHS_DOF:
                self.__data_interface.sleep()
                continue

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
        """Shutdown the ROS interface."""
        self.__data_interface.shutdown()


def main() -> None:
    """Run the X4 forwarder."""
    robot_maver_x4 = IsaacsimMaverX4()
    try:
        robot_maver_x4.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_maver_x4.shutdown()


if __name__ == "__main__":
    main()
