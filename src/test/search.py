#!/usr/bin/env python2

from pennair2 import core, autopilot, search
import rospy

rospy.init_node("search_test")
mavros = autopilot.Mavros(mavros_prefix="/mavros1")
quad = core.Multirotor(mavros, frequency=10)
quad.takeoff()

search_pattern = search.BoxSearch(quad)
while not search_pattern.is_target():
    search_pattern.run()

quad.land()
