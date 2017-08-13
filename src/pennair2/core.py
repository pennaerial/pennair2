from .autopilot import Autopilot

class UAV:
    def __init__(self, autopilot: Autopilot):
        self.autopilot: Autopilot = autopilot

class Multirotor(UAV):
    def __init__(self, autopilot: Autopilot):
        super().__init__(autopilot)