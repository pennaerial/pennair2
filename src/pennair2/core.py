from .autopilot import Autopilot
from abc import ABCMeta, abstractmethod

class UAV:
    def __init__(self, autopilot, metaclass=ABCMeta):
        """

        :param autopilot: An autopilot object to use.
        :type autopilot: Autopilot
        """
        self.autopilot = autopilot

class Multirotor(UAV):
    def __init__(self, autopilot):
        """

        :param autopilot: The autopilot object to use.
        :type autopilot: Autopilot
        """
        UAV.__init__(self, autopilot)

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