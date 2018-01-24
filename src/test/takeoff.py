from pennair2 import core, autopilot
from roslaunch.scriptapi import ROSLaunch
import rospy

rospy.init_node("node")

mavros = autopilot.Mavros()
quad = core.Multirotor(mavros)

quad.set_position([0,0,10], "fcu_local")