from .autopilot import Autopilot
from abc import ABCMeta, abstractmethod
from math import pi
from timeit import default_timer as timer

import rospy
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Float64


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


    def takeoff(self, takeoff_height=1.0, min_time= 1, max_time= 5): #Min and Max Time(s) it take for the quad to take off
        print("Attempting Takeoff")
        msg = TwistStamped()
        msg.Twist.linear = Vector3(0, 0, 0.25)
        self.velocity = msg

        start = timer()
        time_elapsed = 0
        while(time_elapsed < max_time):
            time_elapsed = timer() - start
            if (self.autopilot.relative_altitude.data > takeoff_height and time_elapsed > min_time):
                break

        msg = TwistStamped()
        msg.Twist.linear = Vector3(0, 0, 0)
        self.velocity = msg


    def land(self):
        msg = TwistStamped()
        msg.Twist.linear = Vector3(0, 0, -0.25)
        self.velocity = msg

    def yaw(self):
        raise NotImplementedError

    @property
    def position(self):
        return self.autopilot.gps_raw

    @position.setter
    def position(self, value):
        if type(value) is PoseStamped:
            self._pos_setpoint = value
            self.autopilot.position_pub.publish(value)
        else:
            print("Error while setting position: Value is not PoseStamped")


    @property
    def velocity(self):
        return self.autopilot.gps_velocity_raw

    @velocity.setter
    def velocity(self, value):
        if type(value) is TwistStamped:
            self._vel_setpoint = value
            self.autopilot.velocity_pub.publish(value)
        else:
            print("Error while setting velocity: Value is not TwistStamped")
