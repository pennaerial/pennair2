# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from autopilot import Autopilot, Mavros
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped, Twist, Vector3
from tf_conversions import transformations
from enum import Enum
from conversions import position_to_numpy




class UAV(object):
    class SetpointMode(Enum):
        POSITION = 1
        VELOCITY = 2
        ACCELERATION = 3

    def __init__(self, autopilot, frequency=30):
        """

        :param autopilot: An autopilot object to use.
        :type autopilot: Autopilot
        """
        __metaclass__ = ABCMeta

        self.frequency = frequency
        self.autopilot = autopilot
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120))  # tf buffer length in seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # type: tf2_ros.TransformListener

        rospy.loginfo("Waiting for autopilot connection.")
        self.wait_for(self.autopilot.is_connected)
        rospy.loginfo("Connected.")

        self._setpoint_pos = None  # type: PoseStamped
        self._setpoint_vel = None  # type: TwistStamped
        self._setpoint_mode = None  # type: UAV.SetpointMode
        self._setpoint_heading = None  # type: int

        # setpoint/control stream loop, must publish setpoints to change into OFFBOARD
        self.loop_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / frequency), self.__control_loop)

    def __control_loop(self, event):
        if not rospy.is_shutdown():
            if self._setpoint_mode is UAV.SetpointMode.POSITION:
                self.autopilot.local_pose = self._setpoint_pos
            elif self._setpoint_mode is UAV.SetpointMode.VELOCITY:
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
        """

        :param value: The desired position setpoint, only yaw component of orientation is used. Can be of type
            PoseStamped, Pose, Point, or an indexable object with 3 integer elements (list, tuple, numpy array etc.)
        :type value: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
        :param frame_id: The name of the frame to use for the message.
        :type frame_id: str
        :param heading: Your desired heading.
        :type heading: int
        """
        if isinstance(value, PoseStamped):
            msg = value
            if frame_id is not None:
                msg.header.frame_id = frame_id
        else:
            msg = PoseStamped()
            msg.header.frame_id = frame_id

            if isinstance(value, Pose):  # if Pose then position and orientation already provided
                msg.pose = value
            else:
                if isinstance(value, Point):
                    msg.pose.position = value
                else:
                    msg.pose.position.x = value[0]
                    msg.pose.position.y = value[1]
                    msg.pose.position.z = value[2]

                if heading is None:
                    heading = self._setpoint_heading
                if heading is None:
                    heading = self.get_heading()
                q = transformations.quaternion_from_euler(0, 0, heading)
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]
        msg.header.stamp = rospy.Time.now()
        self._setpoint_pos = msg
        self._setpoint_mode = UAV.SetpointMode.POSITION

    def get_velocity(self):
        return self.autopilot.local_twist

    def set_velocity(self, value, frame_id=None):
        if isinstance(value, TwistStamped):
            msg = value
            if frame_id is not None:
                msg.header.frame_id = frame_id
        else:
            msg = TwistStamped()
            msg.header.frame_id = frame_id

            if isinstance(value, Twist):
                msg.twist = value
            else:
                if isinstance(value, Vector3):
                    msg.twist.linear = value
                else:
                    if isinstance(value, np.ndarray):
                        value = value.reshape((3, 1))
                        msg.twist.linear.x = value[0, 0]
                        msg.twist.linear.y = value[1, 0]
                        msg.twist.linear.z = value[2, 0]
                    elif isinstance(value, (list, tuple)):
                        msg.twist.linear.x = value[0]
                        msg.twist.linear.y = value[1]
                        msg.twist.linear.z = value[2]
                msg.twist.angular.x = 0
                msg.twist.angular.y = 0
                msg.twist.angular.z = 0
        msg.header.stamp = rospy.Time.now()
        self._setpoint_vel = msg
        self._setpoint_mode = UAV.SetpointMode.VELOCITY

    def distance_to_target(self, target=None, current=None, frame_id=None):
        d = position_to_numpy(self.displacement_from_target(target, current, frame_id))
        return np.linalg.norm(d)

    def displacement_from_target(self, target=None, current=None, frame_id=None):
        if target is None:
            target = self._setpoint_pos
        if current is None:
            current = self.get_position()

        if frame_id is not None:
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
            target = tf2_geometry_msgs.do_transform_pose(target, transform1)
            current = tf2_geometry_msgs.do_transform_pose(current, transform2)
        target = target.pose.position
        current = current.pose.position
        return Vector3(target.x - current.x, target.y - current.y, target.z - current.z)

    def shutdown(self):
        self.loop_timer.shutdown()

    def start(self):
        self.loop_timer.start()


class Multirotor(UAV):
    def __init__(self, autopilot, frequency=30):
        """

        :param autopilot: The autopilot object to use.
        :type autopilot: Autopilot
        """
        UAV.__init__(self, autopilot, frequency=frequency)

    def hover(self):
        self.set_position(self.get_position())

    def takeoff(self, speed=1, target_height=10):
        rospy.loginfo("Waiting for arm.")
        self.wait_for(lambda: self.is_armed)
        self.set_velocity([0, 0, abs(speed)])
        rospy.loginfo("Waiting for offboard.")
        self.wait_for(lambda: self.is_offboard)

        rate = rospy.Rate(self.frequency)
        while self.autopilot.relative_altitude < target_height:
            rate.sleep()
        self.hover()

    def land(self, speed=0.5):
        self.hover()
        rospy.sleep(5)
        self.set_velocity([0, 0, -abs(speed)])
        rate = rospy.Rate(self.frequency)
        while self.is_armed:
            rate.sleep()

    def set_position(self, position, frame_id=None, heading=None, blocking=False, margin=0.5):
        UAV.set_position(self, position, frame_id, heading)
        self.set_position(position)
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()
