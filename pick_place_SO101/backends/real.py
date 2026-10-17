"""Real SO-101 follower through LeRobot (feetech STS3215 bus)."""
import time

import numpy as np

from .. import config
from .base import Backend


class RealBackend(Backend):
    name = "real"

    def __init__(self, port, robot_id=config.ROBOT_ID,
                 max_relative_target=config.MAX_RELATIVE_TARGET):
        self._make = self._factory(port, robot_id, max_relative_target)
        self.robot = self._make()
        self._keys = [f"{j}.pos" for j in config.JOINTS]
        self._connected = False

    @staticmethod
    def _factory(port, robot_id, max_relative_target):
        def make():
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            return SO101Follower(SO101FollowerConfig(
                port=port, id=robot_id, use_degrees=True,
                max_relative_target=max_relative_target))
        return make

    def connect(self, attempts=4):
        # uses the saved calibration; torque comes on at the end of connect().
        # lerobot's configure() writes with a single try, so one dropped packet
        # (e.g. while the cameras are spinning up on the same USB host) kills it.
        for i in range(attempts):
            try:
                self.robot.connect(calibrate=False)
                self._connected = True
                return
            except ConnectionError as e:
                print(f"[real] connect attempt {i + 1}/{attempts} failed: {e}")
                if i == attempts - 1:
                    raise
                try:
                    self.robot.disconnect()
                except Exception:
                    pass
                time.sleep(0.7)
                self.robot = self._make()

    def read(self):
        obs = self.robot.get_observation()
        return np.array([obs[k] for k in self._keys], float)

    def command(self, q6):
        self.robot.send_action({k: float(v) for k, v in zip(self._keys, q6)})

    def disconnect(self):
        if self._connected:
            self.robot.disconnect()          # torque off
            self._connected = False
