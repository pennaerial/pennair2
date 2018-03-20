from pennair2 import core, autopilot
import rospy

rospy.init_node("node")
mavros = autopilot.Mavros()
quad = core.Multirotor(mavros, frequency=10)
quad.takeoff()
quad.set_position([0, 0, 10], blocking=True)
rospy.sleep(3)
quad.land()
