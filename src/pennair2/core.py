from .autopilot import Autopilot
from abc import ABCMeta, abstractmethod

class UAV:
    def __init__(self, autopilot: Autopilot, metaclass=ABCMeta):
        self.autopilot: Autopilot = autopilot

class Multirotor(UAV):
    def __init__(self, autopilot: Autopilot):
        super().__init__(autopilot)

    def takeoff(self):
        raise NotImplementedError

    def land(self):
        raise NotImplementedError

    def yaw(self):
        raise NotImplementedError
