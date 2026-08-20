#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################
# bridge-forwarding adaptation: the deliverable never talks to hardware.
#   * subscribes to the bridge joint_states (sensor_msgs/JointState)
#   * publishes the single torque command to the bridge cmd topic
#   * forwards bridge state as chs_state (odom left empty this phase)
# All bridge topics the deliverable subscribes to are exposed as params.

import numpy as np
import threading

from hex_util_runtime import ns_now

import rclpy
import rclpy.node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboChsStateStamped,
    HexRosRoboChsCtrlStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseJntFull,
    HexDcBaseTwist,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl,
    HexDcRoboChsCtrlMode,
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsStateStamped,
)

from .interface_base import ChassisInterfaceBase

from rclpy.logging import LoggingSeverity


class DataInterface(ChassisInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        # self.__logger.set_level(LoggingSeverity.DEBUG)
        self._shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        super().__init__(name)

        ### rate parameters
        self.__node.declare_parameter('ctrl_rate', 1000.0)
        self.__node.declare_parameter('rate_state', 500.0)
        self._rate_param["ros"] = self.__node.get_parameter('ctrl_rate').value
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### robot parameters
        self.__node.declare_parameter('robot_frame_id', "base_link")
        self._robot_param = {
            "frame_id": self.__node.get_parameter('robot_frame_id').value,
        }

        ### bridge parameters (all bridge topics we subscribe to are params)
        self.__node.declare_parameter('bridge_state_topic', '/joint_states')
        self.__node.declare_parameter('bridge_cmd_topic', '/hex_chs_cmd')
        self.__node.declare_parameter('time_source', 'sim')
        self._bridge_param = {
            "state_topic": self.__node.get_parameter('bridge_state_topic').value,
            "cmd_topic": self.__node.get_parameter('bridge_cmd_topic').value,
            "time_source": self.__node.get_parameter('time_source').value,
        }

        ### publisher — chs_state
        self.__chs_state_pub = self.__node.create_publisher(
            HexRosRoboChsStateStamped,
            'chs_state',
            10,
        )
        ### publisher — bridge torque command (sensor_msgs/JointState, effort)
        self.__bridge_cmd_pub = self.__node.create_publisher(
            JointState,
            self._bridge_param["cmd_topic"],
            10,
        )

        ### subscriber — chs_ctrl
        self.__chs_ctrl_sub = self.__node.create_subscription(
            HexRosRoboChsCtrlStamped,
            'chs_ctrl',
            self.__chs_ctrl_callback,
            10,
        )
        self.__chs_ctrl_sub
        ### subscriber — bridge joint_states
        self.__bridge_state_sub = self.__node.create_subscription(
            JointState,
            self._bridge_param["state_topic"],
            self.__bridge_state_callback,
            10,
        )
        self.__bridge_state_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        return ns_now()

    def now_stamp(self) -> HexDcBaseTime:
        now = self.__node.get_clock().now()
        secs, nsecs = now.seconds_nanoseconds()
        return HexDcBaseTime(secs=secs, nsecs=nsecs)

    ####################
    ### publishers
    ####################
    def pub_chs_state(self, out: HexDcRoboChsStateStamped):
        msg = HexRosRoboChsStateStamped()
        # forward stamp: sim (bridge) or ros (now) per time_source param
        forward_stamp_dc = self.get_forward_stamp()
        msg.header.stamp = Time(
            sec=int(forward_stamp_dc.secs),
            nanosec=int(forward_stamp_dc.nsecs),
        )
        msg.header.frame_id = out.header.frame_id

        jnt = out.chs_state.jnt
        msg.chs_state.jnt.header.stamp = Time(
            sec=int(forward_stamp_dc.secs),
            nanosec=int(forward_stamp_dc.nsecs),
        )
        msg.chs_state.jnt.header.frame_id = out.header.frame_id
        msg.chs_state.jnt.name = self._joint_names
        msg.chs_state.jnt.position = \
            np.asarray(jnt.position, dtype=np.float64).tolist()
        msg.chs_state.jnt.velocity = \
            np.asarray(jnt.velocity, dtype=np.float64).tolist()
        msg.chs_state.jnt.effort = \
            np.asarray(jnt.effort, dtype=np.float64).tolist()

        # odom is left empty this phase (default zeros)

        self.__chs_state_pub.publish(msg)

    def pub_bridge_cmd(self, names: list, effort: np.ndarray):
        msg = JointState()
        forward_stamp = self.get_forward_stamp()
        msg.header.stamp = Time(
            sec=int(forward_stamp.secs),
            nanosec=int(forward_stamp.nsecs),
        )
        msg.name = names
        msg.effort = np.asarray(effort, dtype=np.float64).tolist()
        self.__bridge_cmd_pub.publish(msg)

    ####################
    ### subscribers
    ####################
    def __bridge_state_callback(self, msg: JointState):
        stamp = HexDcBaseTime(
            secs=int(msg.header.stamp.sec),
            nsecs=int(msg.header.stamp.nanosec),
        )
        self.set_bridge_state(
            msg.name, msg.position, msg.velocity, msg.effort, stamp)

    def __chs_ctrl_callback(self, msg: HexRosRoboChsCtrlStamped):
        self._chs_ctrl_deque.append(self.__chs_ctrl_msg_to_dc(msg))

    @staticmethod
    def __chs_ctrl_msg_to_dc(
            msg: HexRosRoboChsCtrlStamped) -> HexDcRoboChsCtrlStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        chs_msg = msg.chs_ctrl
        chs_ctrl = HexDcRoboChsCtrl(
            ctrl_mode=HexDcRoboChsCtrlMode(int(chs_msg.ctrl_mode)),
            jnt=DataInterface.__jnt_to_dc(chs_msg.jnt),
            vel=HexDcBaseTwist(
                linear=HexDcBaseVector3(
                    x=chs_msg.vel.linear.x,
                    y=chs_msg.vel.linear.y,
                    z=chs_msg.vel.linear.z,
                ),
                angular=HexDcBaseVector3(
                    x=chs_msg.vel.angular.x,
                    y=chs_msg.vel.angular.y,
                    z=chs_msg.vel.angular.z,
                ),
            ),
        )

        return HexDcRoboChsCtrlStamped(
            header=header,
            chs_ctrl=chs_ctrl,
        )

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64),
        )
