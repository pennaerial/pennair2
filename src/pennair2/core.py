from .autopilot import Autopilot, Mavros
from abc import ABCMeta, abstractmethod
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped, Twist, Vector3
from tf_conversions import transformations

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
import threading

class UAV:
    def __init__(self, autopilot, frequency=30):
        """

        :param autopilot: An autopilot object to use.
        :type autopilot: Autopilot
        """
        __metaclass__ = ABCMeta

        self.autopilot = autopilot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120))  # tf buffer length in seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # type: tf2_ros.TransformListener

        self._setpoint_pos = autopilot.local_pose
        self._setpoint_vel = None
        self._setpoint_mode = None  # None, "POSITION", "VELOCITY"
        self._setpoint_heading = autopilot.heading



        rospy.loginfo("Waiting for autopilot connection.")
        self.wait_for(self.autopilot.is_connected)
        rospy.loginfo("Waiting for arm.")
        self.wait_for(lambda: self.is_armed)

        self.set_velocity([0, 0, 0])
        loop_timer = rospy.Timer(rospy.Duration.from_sec(1.0/frequency), self.__control_loop)
        loop_timer.run()

        rospy.loginfo("Waiting for offboard mode.")
        self.wait_for(lambda: self.is_offboard)
        rospy.loginfo("Armed and in offboard!")

    def __control_loop(self, event):
        if not rospy.is_shutdown():
            if self.is_armed and self.is_offboard:
                if self._setpoint_mode is not None:
                    if self._setpoint_mode == "POSITION":
                        self.autopilot.local_pose = self._setpoint_pos
                    elif self._setpoint_mode == "VELOCITY":
                        self.autopilot.local_twist = self._setpoint_vel

    @property
    def is_armed(self):
        if isinstance(self.autopilot, Mavros):
            return self.autopilot.state.armed

    @property
    def is_offboard(self):
        if isinstance(self.autopilot, Mavros):
            return self.autopilot.state.mode == "OFFBOARD"

    def wait_for(self, fun, rate=30, wait_val=True):
        while not fun() == wait_val:
            rospy.sleep(1.0/rate)

    def get_gps(self):
        return self.autopilot.global_global

    def get_heading(self):
        return math.pi * self.autopilot.heading / 180

    def get_relative_altitude(self):
        return self.autopilot.relative_altitude

    def get_position(self, utm=False):
        if utm:
            return self.autopilot.global_local
        else:
            return self.autopilot.local_pose

    def set_position(self, value, frame_id=None, heading=None):
        if isinstance(value, PoseStamped):
            value.header.frame_id = frame_id
            self._setpoint_pos = value
        else:
            msg = PoseStamped()
            msg.header.frame_id = frame_id

            if type(value) is Pose:  # if Pose then position and orientation already provided
                msg.pose = value
            else:
                if type(value) is Point:
                    msg.pose.position = value
                else:
                    msg.pose.position.x = value[0]
                    msg.pose.position.y = value[1]
                    msg.pose.position.z = value[2]

                if heading is None:
                    heading = self._setpoint_heading
                q = transformations.quaternion_from_euler(0, 0, heading)
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]

            self._setpoint_pos = msg
        self._setpoint_mode = "POSITION"

    def get_velocity(self):
        return self.autopilot.local_twist

    def set_velocity(self, value, frame_id=None):
        if type(value) is TwistStamped:
            value.header.frame_id = frame_id
            self._setpoint_vel = value
        else:
            msg = TwistStamped()
            msg.header.frame_id = frame_id

            if type(value) is Twist:
                msg.twist = value
            else:
                if type(value) is Vector3:
                    msg.twist.linear = value
                else:
                    msg.twist.linear.x = value[0]
                    msg.twist.linear.y = value[1]
                    msg.twist.linear.z = value[2]
                msg.twist.angular.x = 0
                msg.twist.angular.y = 0
                msg.twist.angular.z = 0
            self._setpoint_vel = value
        self._setpoint_mode = "VELOCITY"

    def pose_distance(self, frame_id, target=None, current=None):
        transform1 = self.tf_buffer.lookup_transform(frame_id,
                                                     target.header.frame_id,
                                                     rospy.Time(0),
                                                     rospy.Duration(1)
                                                     )

        transform2 = self.tf_buffer.lookup_transform(frame_id,
                                                     current.header.frame_id,
                                                     rospy.Time(0),
                                                     rospy.Duration(1)
                                                     )

        target = tf2_geometry_msgs.do_transform_pose(target, transform1).pose.position
        current = tf2_geometry_msgs.do_transform_pose(current, transform2).pose.position
        return math.sqrt((target.x - current.x)**2 + (target.y - current.y)**2 + (target.z - current.z)**2)


class Multirotor(UAV):
    def __init__(self, autopilot, frequency=30):
        """

        :param autopilot: The autopilot object to use.
        :type autopilot: Autopilot
        """
        UAV.__init__(self, autopilot, frequency=frequency)

    def hover(self):
        self.set_position(self.get_position())

    def takeoff(self, speed=0.5, target_height=10):
        self.set_velocity([0, 0, abs(speed)])
        rate = rospy.Rate(100)
        while self.autopilot.relative_altitude < target_height:
            rate.sleep()
        self.hover()

    def land(self, speed=0.5):
        self.hover()
        self.set_velocity([0, 0, -abs(speed)])
        rate = rospy.Rate(100)
        while self.autopilot.relative_altitude > 0:
            rate.sleep()