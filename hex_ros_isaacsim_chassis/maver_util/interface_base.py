#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Define the transport abstraction for the Isaac Sim Maver X4 bridge."""

from abc import ABC, abstractmethod
from collections import deque
from threading import Lock
from typing import Any, Optional

import numpy as np


MAVER_JOINT_NAMES = [
    "joint_wheel1", "joint_yaw1",
    "joint_wheel2", "joint_yaw2",
    "joint_wheel3", "joint_yaw3",
    "joint_wheel4", "joint_yaw4",
]
MAVER_WHEEL_INDEX = np.array([0, 2, 4, 6], dtype=np.int64)
MAVER_YAW_INDEX = np.array([1, 3, 5, 7], dtype=np.int64)


class ChassisInterfaceBase(ABC):
    """Define ROS-independent transport services for the Maver X4 node.

    The concrete ROS implementation owns the callback thread, while the
    control node consumes queued commands from its fixed-rate loop.
    """

    def __init__(self, name: str = "unknown") -> None:
        """Initialize transport state.

        Args:
            name: Logical name used by the concrete transport implementation.

        """
        self._name = name
        self._rate_param = {}
        self._chs_ctrl_deque = deque(maxlen=100)
        self._joint_names = []
        self._joint_positions = np.zeros(8, dtype=np.float64)
        # False until a complete, name-validated state frame is accepted.
        self._joint_state_valid = False
        # None until the first valid Bridge name order is received.
        self._canonical_to_bridge_index = None
        # Protect the state snapshot shared by ROS callbacks and the control loop.
        self._joint_state_lock = Lock()

    def get_rate_param(self) -> dict:
        """Return the configured control-rate parameters.

        Returns:
            A dictionary containing the transport-specific rate values.

        """
        return self._rate_param

    def set_joint_names(self, names: list) -> None:
        """Store a valid Bridge joint-name sequence.

        Args:
            names: Joint names in the order supplied by Isaac Sim.

        """
        self._set_bridge_mapping(names)

    def get_joint_names(self) -> list:
        """Return the latest Bridge joint names.

        Returns:
            A copy of the cached Bridge joint-name sequence.

        """
        with self._joint_state_lock:
            return list(self._joint_names)

    def _set_bridge_mapping(self, names: list) -> bool:
        """Build the canonical-to-Bridge index mapping.

        Args:
            names: Joint names in one Bridge message.

        Returns:
            ``True`` when all Maver names are present exactly once.

        """
        if not self._valid_joint_names(names):
            return False
        # Build indices from names because Bridge array order is not canonical.
        bridge_index = {name: index for index, name in enumerate(names)}
        with self._joint_state_lock:
            # Keep raw Bridge names only for the final command conversion.
            self._joint_names = list(names)
            self._canonical_to_bridge_index = np.asarray(
                [bridge_index[name] for name in MAVER_JOINT_NAMES],
                dtype=np.int64)
        return True

    def set_joint_state(self, names: list, positions) -> bool:
        """Update canonical joint positions from one Bridge state frame.

        Args:
            names: Joint names contained in the incoming state message.
            positions: Joint positions corresponding to ``names``.

        Returns:
            ``True`` when the state was complete and accepted.

        """
        if len(positions) != len(names):
            with self._joint_state_lock:
                # Reject incomplete frames instead of guessing missing joints.
                self._joint_state_valid = False
            return False
        if not self._valid_joint_names(names):
            with self._joint_state_lock:
                # Reject incomplete frames instead of guessing missing joints.
                self._joint_state_valid = False
            return False

        # Convert this frame to the wheel/yaw order used by hex_ros_robot.
        position_by_name = dict(zip(names, positions))
        canonical_positions = np.asarray(
            [position_by_name[name] for name in MAVER_JOINT_NAMES],
            dtype=np.float64)
        bridge_index = {name: index for index, name in enumerate(names)}
        with self._joint_state_lock:
            # Update state and its matching mapping as one snapshot.
            self._joint_names = list(names)
            self._canonical_to_bridge_index = np.asarray(
                [bridge_index[name] for name in MAVER_JOINT_NAMES],
                dtype=np.int64)
            self._joint_positions = canonical_positions
            self._joint_state_valid = True
        return True

    def has_valid_joint_state(self) -> bool:
        """Return whether a complete Bridge state has been received.

        Returns:
            ``True`` when the cached state can safely be used for IK.

        """
        with self._joint_state_lock:
            return self._joint_state_valid

    def get_joint_positions(self) -> np.ndarray:
        """Return the latest joint positions in canonical ROS order.

        Returns:
            A copy of the cached position array ordered as wheel/yaw pairs.

        """
        with self._joint_state_lock:
            return self._joint_positions.copy()

    def to_bridge_order(
        self, values: np.ndarray, names: Optional[list] = None
    ) -> Optional[np.ndarray]:
        """Reorder a canonical joint array for a Bridge name sequence.

        Args:
            values: Eight values in canonical ROS order.
            names: Target Bridge names. The latest cached names are used when
                omitted.

        Returns:
            Values in Bridge order, or ``None`` when mapping is unavailable.

        """
        values = np.asarray(values, dtype=np.float64)
        if values.size != len(MAVER_JOINT_NAMES):
            return None
        if names is None:
            with self._joint_state_lock:
                if self._canonical_to_bridge_index is None:
                    return None
                bridge_index = self._canonical_to_bridge_index.copy()
        else:
            if not self._valid_joint_names(names):
                return None
            bridge_index = np.asarray(
                [names.index(name) for name in MAVER_JOINT_NAMES],
                dtype=np.int64)
        # Restore the order expected by the receiving Bridge endpoint.
        bridge_values = np.empty_like(values)
        bridge_values[bridge_index] = values
        return bridge_values

    @staticmethod
    def _valid_joint_names(names: list) -> bool:
        """Return whether names contain the complete Maver joint set.

        Args:
            names: Joint names to validate.

        Returns:
            ``True`` when names contain each canonical joint exactly once.

        """
        return (len(names) == len(MAVER_JOINT_NAMES)
                and len(set(names)) == len(names)
                and set(names) == set(MAVER_JOINT_NAMES))

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
            # The fixed-rate loop needs the newest command, not stale inputs.
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
                          effort: np.ndarray,
                          position: np.ndarray = None) -> None:
        """Publish one complete Isaac Sim joint command.

        Args:
            names: Joint names defining the array order.
            velocity: Joint velocity targets in Bridge order.
            effort: Joint effort feed-forward values.
            position: Optional joint position targets.

        """
        raise NotImplementedError
