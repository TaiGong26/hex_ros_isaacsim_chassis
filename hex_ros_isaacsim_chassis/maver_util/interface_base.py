"""Transport abstraction for the Isaac Sim X4 forwarder.

Copyright 2026 Dong Zhaorui. All rights reserved.
Author: Dong Zhaorui 847235539@qq.com
Date: 2026-08-31
"""

from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Optional

import numpy as np


class ChassisInterfaceBase(ABC):
    """Define the ROS-independent interface used by the X4 node."""

    def __init__(self, name: str = "unknown") -> None:
        self._name = name
        self._rate_param = {}
        self._chs_ctrl_deque = deque(maxlen=100)
        self._joint_names = []
        # Isaac Sim bridge order is authoritative.  For Maver this is
        # [yaw1, yaw2, yaw3, yaw4, wheel1, wheel2, wheel3, wheel4].
        self._joint_positions = np.zeros(8, dtype=np.float64)

    def get_rate_param(self) -> dict:
        """Return control rate parameters."""
        return self._rate_param

    def set_joint_names(self, names: list) -> None:
        """Store the first bridge joint-name frame without reordering it."""
        if not self._joint_names and names:
            self._joint_names = list(names)

    def get_joint_names(self) -> list:
        """Return bridge joint names in their original order."""
        return list(self._joint_names)

    def set_joint_state(self, names: list, positions) -> None:
        """Capture names and positions in exactly the bridge's name order."""
        if not names:
            return
        if not self._joint_names:
            self._joint_names = list(names)
        if len(positions) != len(names):
            return
        position_by_name = dict(zip(names, positions))
        if all(name in position_by_name for name in self._joint_names):
            self._joint_positions = np.asarray(
                [position_by_name[name] for name in self._joint_names],
                dtype=np.float64)

    def get_joint_positions(self) -> np.ndarray:
        """Return the latest joint positions in bridge order."""
        return self._joint_positions.copy()

    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        """Pop one command, optionally discarding older commands."""
        if not dq:
            return None
        if latest:
            value = dq[-1]
            dq.clear()
            return value
        return dq.popleft()

    def get_chs_ctrl(self, latest: bool = False):
        """Return a queued chassis command."""
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
                          effort: np.ndarray,
                          position: np.ndarray = None) -> None:
        """Publish one complete bridge joint command."""
        raise NotImplementedError
