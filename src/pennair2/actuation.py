# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import rospy
from mavros_msgs.msg import OverrideRCIn


class RCOverride:
    def __init__(self, mavros_prefix="/mavros"):
        # type: (str) -> None
        self.actuator_pub = rospy.Publisher(mavros_prefix + "/rc/override", OverrideRCIn, queue_size=10)
        self.channels = [OverrideRCIn.CHAN_RELEASE] * 8

    def set_pwm(self, val, channel):
        # type: (int, int) -> None
        msg = OverrideRCIn()  # type: OverrideRCIn
        self.channels[channel-1] = val
        msg.channels = self.channels
        self.actuator_pub.publish(msg)