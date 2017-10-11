from .autopilot import Autopilot
from abc import ABCMeta, abstractmethod

class UAV:
    def __init__(self, autopilot: Autopilot, metaclass=ABCMeta):
        self.autopilot: Autopilot = autopilot

class Multirotor(UAV):
    def __init__(self, autopilot: Autopilot):
        super().__init__(autopilot)

        self._pos_setpoint = None
        self._vel_setpoint = None
        self._acc_setpoint = None

    def takeoff(self):
        raise NotImplementedError

    def land(self):
        raise NotImplementedError

    def yaw(self):
        raise NotImplementedError

    @property
    def position(self):
        raise NotImplementedError

    @position.setter
    def position(self):
        raise NotImplementedError

    @property
    def velocity(self):
        raise NotImplementedError

    @velocity.setter
    def velocity(self):
        raise NotImplementedError