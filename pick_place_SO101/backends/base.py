"""Backend ABC. Everything crossing this boundary is LeRobot units:
5 arm joints in degrees, gripper in percent (0 closed .. 100 open)."""
from abc import ABC, abstractmethod

import numpy as np


class Backend(ABC):
    name = "base"

    @abstractmethod
    def connect(self): ...

    @abstractmethod
    def read(self) -> np.ndarray:
        """Measured joint state (6,)."""

    @abstractmethod
    def command(self, q6: np.ndarray) -> None:
        """Send an absolute joint target (6,)."""

    def advance(self, dt: float):
        """Sim backends step physics here; real hardware ignores it."""

    def set_markers(self, target_p=None, tcp_p=None):
        """Optional: show the commanded TCP target in a viewer."""

    def sync_viewer(self):
        pass

    def viewer_azimuth(self):
        return None

    def running(self) -> bool:
        return True

    @abstractmethod
    def disconnect(self): ...
