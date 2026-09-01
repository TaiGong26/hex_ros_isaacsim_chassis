#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Define the transport abstraction for the Isaac Sim Trigger A3 bridge."""

from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Optional

import numpy as np


class ChassisInterfaceBase(ABC):
    """Define ROS-independent transport services for the Trigger A3 node.

    The concrete ROS implementation owns subscriptions and callback
    processing, while the control node consumes commands from its loop.
    """

    def __init__(self, name: str = "unknown") -> None:
        """Initialize the transport state.

        Args:
            name: Logical name used by the concrete transport implementation.

        """
        self._name = name
        self._rate_param = {}
        self._chs_ctrl_deque = deque(maxlen=100)

    def get_rate_param(self) -> dict:
        """Return the configured control-rate parameters.

        Returns:
            A dictionary containing transport-specific rate values.

        """
        return self._rate_param

    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        """Retrieve one queued item using the requested queue policy.

        Args:
            dq: Queue from which to retrieve an item.
            latest: If true, discard older items and return the newest item.

        Returns:
            The selected item, or ``None`` when the queue is empty.

        """
        if not dq:
            return None
        if latest:
            value = dq[-1]
            dq.clear()
            return value
        return dq.popleft()

    def get_chs_ctrl(self, latest: bool = False):
        """Return one queued chassis command.

        Args:
            latest: Whether to discard stale commands and return the newest.

        Returns:
            A chassis control message, or ``None`` if no command is queued.

        """
        return self.deque_helper(self._chs_ctrl_deque, latest)

    @abstractmethod
    def ok(self) -> bool:
        """Return whether ROS is active."""
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        """Release ROS resources."""
        raise NotImplementedError

    @abstractmethod
    def sleep(self) -> None:
        """Sleep for one control period."""
        raise NotImplementedError

    @abstractmethod
    def logi(self, msg, *args, **kwargs) -> None:
        """Write an informational log."""
        raise NotImplementedError

    @abstractmethod
    def logd(self, msg, *args, **kwargs) -> None:
        """Write a debug log."""
        raise NotImplementedError

    @abstractmethod
    def logw(self, msg, *args, **kwargs) -> None:
        """Write a warning log."""
        raise NotImplementedError

    @abstractmethod
    def loge(self, msg, *args, **kwargs) -> None:
        """Write an error log."""
        raise NotImplementedError

    @abstractmethod
    def pub_joint_command(self, names: list, velocity: np.ndarray,
                          effort: np.ndarray) -> None:
        """Publish one complete Isaac Sim joint command.

        Args:
            names: Joint names defining the array order.
            velocity: Joint velocity targets in Bridge order.
            effort: Joint effort feed-forward values.

        """
        raise NotImplementedError
