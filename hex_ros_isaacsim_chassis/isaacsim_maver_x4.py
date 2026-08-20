#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-07-31
################################################################
# Isaac Sim bridge-forwarding chassis node (Maver X4).
#
# Subscribes to chs_ctrl (hex_ros_msgs), computes the chassis torque locally
# mirroring hex_ros_sim_maver_x4/mujoco_sim.py, publishes the single torque to
# the bridge cmd topic, and forwards the bridge joint_states as chs_state
# (odom left empty this phase).
#
# Joint order (ROS canonical): joint_wheel1, joint_yaw1, ..., joint_wheel4,
# joint_yaw4 — same order as the bridge joint_states (dynamic from the sim).

import os
import sys

import numpy as np

scrpit_path = os.path.abspath(os.path.dirname(__file__))
if scrpit_path not in sys.path:
    sys.path.append(scrpit_path)
from maver_util import DataInterface

from hex_util_ros import angle_norm, HexFricUtil

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl,
    HexDcRoboChsCtrlMode,
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsState,
    HexDcRoboChsStateStamped,
)
from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseJntState,
    HexDcBaseJntFull,
    HexDcBaseVector3,
    HexDcBaseTwist,
)


# Maver 8-motor canonical joint names (ROS order, fallback before the bridge
# joint_states arrives)
JOINT_STATE_NAME = [
    "joint_wheel1", "joint_yaw1",
    "joint_wheel2", "joint_yaw2",
    "joint_wheel3", "joint_yaw3",
    "joint_wheel4", "joint_yaw4",
]

CHS_DOF = 8

# Default gains used to seed the initial MIT command (mirrors mujoco_sim.py)
CHS_KP_DEFAULT = np.array([0.0] * 8)
CHS_KD_DEFAULT = np.array([10.0] * 8)

# VEL mode kinematic params (mirrors mujoco_sim.py)
CHS_PARAMS = {
    "wheel_radius": 0.0625,
    "track_width": 0.28,
    "wheel_base": 0.424,
    "bias": 0.02,
}
ROS_YAW = np.array([1, 3, 5, 7])
ROS_WHEEL = np.array([0, 2, 4, 6])


class IsaacsimMaverX4:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("hex_ros_isaacsim_maver_x4")

        ### parameters
        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(f"rate_state: {rate_param['state']} hz")
        self.__data_interface.logi(
            f"robot_frame_id: {robot_param['frame_id']}")
        self.__data_interface.logi(f"robot_type: {robot_param['robot_type']}")

        ### joint names (dynamic from the bridge, canonical fallback)
        self.__data_interface.set_dofs(CHS_DOF, JOINT_STATE_NAME)

        ### friction compensation (mirrors mujoco_sim.py)
        self.__fric_util = HexFricUtil(
            fc=np.array([0.05] * CHS_DOF),
            fv=np.array([0.0015] * CHS_DOF),
            fo=np.array([0.0] * CHS_DOF),
            k=np.array([100.0] * CHS_DOF),
        )

        ### VEL mode kinematics (mirrors mujoco_sim.py)
        self.__wheel_radius_inv = 1.0 / CHS_PARAMS["wheel_radius"]
        self.__bias_inv = 1.0 / CHS_PARAMS["bias"]
        self.__wheel_distance = 0.5 * np.ones(4) * np.sqrt(
            CHS_PARAMS["track_width"]**2 + CHS_PARAMS["wheel_base"]**2)
        temp_beta = np.arctan2(CHS_PARAMS["track_width"],
                               CHS_PARAMS["wheel_base"])
        self.__beta = np.array(
            [temp_beta, np.pi - temp_beta, temp_beta - np.pi, -temp_beta])
        self.__jac_inv = np.empty((CHS_DOF, 3))

        ### derived
        self.__state_decim = max(
            1,
            int(round(rate_param["ros"] / rate_param["state"])),
        )
        self.__robot_frame_id = robot_param["frame_id"]

        ### initial seed ctrl: MIT hold with kd-only damping (mirrors the sim)
        self.__cur_ctrl = HexDcRoboChsCtrlStamped(
            header=HexDcBaseHeader(
                stamp=self.__data_interface.now_stamp(),
                frame_id=self.__robot_frame_id,
            ),
            chs_ctrl=HexDcRoboChsCtrl(
                ctrl_mode=HexDcRoboChsCtrlMode.MIT,
                jnt=HexDcBaseJntFull(
                    pos=np.zeros(CHS_DOF),
                    vel=np.zeros(CHS_DOF),
                    eff=np.zeros(CHS_DOF),
                    kp=CHS_KP_DEFAULT.copy(),
                    kd=CHS_KD_DEFAULT.copy(),
                    lim_vel=np.zeros(CHS_DOF),
                    lim_acc=np.zeros(CHS_DOF),
                ),
                vel=HexDcBaseTwist(
                    linear=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
                    angular=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
                ),
            ),
        )

    ####################
    ### torque computation (mirrors mujoco_sim.py)
    ####################
    def __calc_jac_inv(self, cur_pos: np.ndarray) -> np.ndarray:
        """Rebuild the 8x3 inverse jacobian from the current yaw angles.

        Rows are in ROS order [wheel1, yaw1, wheel2, yaw2, ...].
        """
        theta = cur_pos[ROS_YAW]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sin_theta_beta = np.sin(theta - self.__beta)
        cos_theta_beta = np.cos(theta - self.__beta)

        mat_wheel = np.column_stack(
            (cos_theta, sin_theta,
             self.__wheel_distance * sin_theta_beta)) * self.__wheel_radius_inv
        mat_yaw = np.column_stack(
            (-sin_theta, cos_theta,
             self.__wheel_distance * cos_theta_beta - CHS_PARAMS["bias"])) \
            * self.__bias_inv

        self.__jac_inv[ROS_WHEEL, :] = mat_wheel
        self.__jac_inv[ROS_YAW, :] = mat_yaw
        return self.__jac_inv

    def __motor_cmd(self, ctrl_vel) -> np.ndarray:
        """motor = jac_inv @ [vx, vy, omega] (ROS order)."""
        cmd = np.array(
            [ctrl_vel.linear.x, ctrl_vel.linear.y, ctrl_vel.angular.z])
        return self.__jac_inv @ cmd

    def __apply_chs_ctrl(
        self,
        ctrl: HexDcRoboChsCtrlStamped,
        cur_pos: np.ndarray,
        cur_vel: np.ndarray,
    ) -> np.ndarray:
        chs_ctrl = ctrl.chs_ctrl
        mode = chs_ctrl.ctrl_mode
        jnt = chs_ctrl.jnt
        comp = self.__fric_util(cur_vel)

        if mode == HexDcRoboChsCtrlMode.MIT:
            err_pos = angle_norm(jnt.pos - cur_pos)
            err_vel = jnt.vel - cur_vel
            tau_cmds = jnt.kp * err_pos + jnt.kd * err_vel + jnt.eff + comp

        elif mode == HexDcRoboChsCtrlMode.VEL:
            self.__calc_jac_inv(cur_pos)
            motor_vel = self.__motor_cmd(chs_ctrl.vel)
            tau_cmds = jnt.kd * (motor_vel - cur_vel) + jnt.eff + comp

        else:
            raise ValueError(f"Unsupported chassis control mode: {mode}")

        return tau_cmds

    ####################
    ### state forwarding
    ####################
    def __build_chs_state(
        self,
        cur_pos: np.ndarray,
        cur_vel: np.ndarray,
        cur_eff: np.ndarray,
    ) -> HexDcRoboChsStateStamped:
        """Build chs_state from the bridge state (odom left empty)."""
        return HexDcRoboChsStateStamped(
            header=HexDcBaseHeader(
                stamp=self.__data_interface.now_stamp(),
                frame_id=self.__robot_frame_id,
            ),
            chs_state=HexDcRoboChsState(
                jnt=HexDcBaseJntState(
                    position=cur_pos,
                    velocity=cur_vel,
                    effort=cur_eff,
                ),
                # odom is left empty this phase
            ),
        )

    def run(self):
        state_count = 0
        while self.__data_interface.ok():
            # 1. drain to the latest control frame
            ctrl = self.__data_interface.get_chs_ctrl(latest=True)
            if ctrl is not None:
                self.__cur_ctrl = ctrl

            # 2. compute the single torque from the latest bridge state and
            #    publish it to the bridge cmd topic at ctrl_rate
            cur_pos, cur_vel, cur_eff = self.__data_interface.get_chs_state()
            tau_cmds = self.__apply_chs_ctrl(self.__cur_ctrl, cur_pos, cur_vel)
            self.__data_interface.pub_bridge_cmd(
                self.__data_interface.get_joint_names(), tau_cmds)

            # 3. forward bridge state as chs_state at rate_state
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0
                chs_state = self.__build_chs_state(cur_pos, cur_vel, cur_eff)
                self.__data_interface.pub_chs_state(chs_state)

            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    isaacsim_maver = IsaacsimMaverX4()
    try:
        isaacsim_maver.run()
    except KeyboardInterrupt:
        pass
    finally:
        isaacsim_maver.shutdown()


if __name__ == '__main__':
    main()
