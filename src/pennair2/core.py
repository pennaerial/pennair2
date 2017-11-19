from .autopilot import Autopilot
from abc import ABCMeta, abstractmethod

import rospy
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Twist, Vector3


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


        self._position = None


    def takeoff(self, max_angle):
        print("Attempting Takeoff")
        msg = TwistStamped()
        msg.Twist.linear = Vector3(0, 0, 0.5)
        self.velocity = msg

    def land(self):
        raise NotImplementedError

    def yaw(self):
        raise NotImplementedError

    @property
    def position(self):
        return self.autopilot.gps_raw

    @position.setter
    def position(self, value):
        try:
            if type(value) is PoseStamped:
                self.autopilot.velocity_pub.publish(value)
            else:
                print("Error while setting position: Value is not PoseStamped")

    @property
    def velocity(self):
        return self.autopilot.gps_velocity_raw

    @velocity.setter
    def velocity(self, value):
        try:
            if type(value) is TwistStamped:
                self.autopilot.velocity_pub.publish(value)
            else:
                print("Error while setting velocity: Value is not TwistStamped")
