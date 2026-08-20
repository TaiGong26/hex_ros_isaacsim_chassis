#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################
# bridge-forwarding adaptation: joint names are taken dynamically from the
# bridge joint_states `name` field; a generated fallback is used before the
# first bridge state arrives.

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

import numpy as np

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTime
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsStateStamped,
)


class ChassisInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        self._name = name

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}
        self._bridge_param = {}

        ### rx msg queues
        self._chs_ctrl_deque = deque(maxlen=100)

        ### bridge state cache (latest joint_states from the bridge)
        self._bridge_stamp = HexDcBaseTime()
        self._bridge_names = []
        self._bridge_state = {}  # name -> (pos, vel, eff)

        ### joint names (dynamic from bridge; canonical fallback before arrival)
        self._chs_dof = 8
        self._joint_names = []

        print(f"#### ChassisInterfaceBase init: {self._name} ####")

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("ChassisInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("ChassisInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("ChassisInterfaceBase.sleep")

    @abstractmethod
    def now_ns(self) -> int:
        raise NotImplementedError("ChassisInterfaceBase.now_ns")

    @abstractmethod
    def now_stamp(self) -> HexDcBaseTime:
        raise NotImplementedError("ChassisInterfaceBase.now_stamp")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_robot_param(self) -> dict:
        return self._robot_param

    def get_bridge_param(self) -> dict:
        return self._bridge_param

    def set_dofs(self, chs_dof: int, joint_names: list):
        """Chassis DOF + canonical fallback joint names."""
        self._chs_dof = chs_dof
        self._joint_names = list(joint_names)

    ####################
    ### joint names & bridge state cache
    ####################
    def set_bridge_state(self, names, position, velocity, effort, stamp):
        """Cache the latest bridge joint_states (called from the ROS callback)."""
        self._bridge_stamp = stamp
        self._bridge_names = list(names)
        self._bridge_state = {
            n: (float(position[i]), float(velocity[i]), float(effort[i]))
            for i, n in enumerate(self._bridge_names)
        }
        if len(self._bridge_names) >= self._chs_dof:
            self._joint_names = list(self._bridge_names[:self._chs_dof])

    def get_joint_names(self) -> list:
        return list(self._joint_names)

    def get_bridge_stamp(self) -> HexDcBaseTime:
        return self._bridge_stamp

    def get_forward_stamp(self) -> HexDcBaseTime:
        """Header stamp for forwarded msgs: bridge sim time or ROS now."""
        if self._bridge_param.get("time_source", "sim") == "ros":
            return self.now_stamp()
        return self.get_bridge_stamp()

    def get_chs_state(self) -> np.ndarray:
        """(pos, vel, eff) of chassis joints in joint_names order."""
        pos = np.zeros(self._chs_dof)
        vel = np.zeros(self._chs_dof)
        eff = np.zeros(self._chs_dof)
        for i, n in enumerate(self._joint_names):
            v = self._bridge_state.get(n)
            if v is not None:
                pos[i], vel[i], eff[i] = v
        return pos, vel, eff

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_chs_state(self, out: HexDcRoboChsStateStamped):
        raise NotImplementedError("ChassisInterfaceBase.pub_chs_state")

    @abstractmethod
    def pub_bridge_cmd(self, names: list, effort: np.ndarray):
        raise NotImplementedError("ChassisInterfaceBase.pub_bridge_cmd")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # chs ctrl
    def get_chs_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboChsCtrlStamped]:
        return self.deque_helper(self._chs_ctrl_deque, latest)
