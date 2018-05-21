# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from autopilot import Autopilot, Mavros
from abc import ABCMeta
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped, Twist, Vector3
from enum import Enum

from pennair2 import conversions
from .conversions import to_numpy, to_pose_stamped
from pennair2.PID import PID


class UAV(object):
    class SetpointMode(Enum):
        POSITION = 1
        VELOCITY = 2
        ACCELERATION = 3

    def __init__(self, autopilot, frequency=30):
        # type: (Autopilot, float) -> None
        """

        :param autopilot: An autopilot object to use.
        :type autopilot: Autopilot
        """
        __metaclass__ = ABCMeta

        self.frequency = float(frequency)
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

    def wait(self, seconds, rate=30):
        rospy.sleep(seconds / rate)

    def wait_for(self, fun, rate=30, wait_val=True):
        while not fun() == wait_val:
            rospy.sleep(1.0 / rate)

    def get_gps(self):
        return self.autopilot.global_global

    def get_heading(self):
        return math.pi * self.autopilot.heading / 180

    def get_relative_altitude(self):
        return self.autopilot.relative_altitude

    def get_pose(self, utm=False, fmt="pose"):
        # type: (bool, str) -> PoseStamped
        if fmt is "pose":
            if utm:
                pose_covariance = self.autopilot.global_local
                return PoseStamped(pose_covariance.header, pose_covariance.pose.pose)
            else:
                return self.autopilot.local_pose

    def get_position(self, utm=False):
        if utm:
            pose_covariance = self.autopilot.global_local
            pose_stamped = PoseStamped(pose_covariance.header, pose_covariance.pose.pose)
            return conversions.to_numpy(pose_stamped), pose_stamped.header.frame_id
        else:
            pose_stamped = self.autopilot.local_pose
            return to_numpy(pose_stamped)

    def set_position(self, value, frame_id="map", heading=None):
        """

        :param value: The desired position setpoint, only yaw component of orientation is used. Can be of type
            PoseStamped, Pose, Point, or an indexable object with 3 integer elements (list, tuple, numpy array etc.)
        :type value: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
        :param frame_id: The name of the frame to use for the message.
            Will only be applied if value is not already PoseStamped.
        :type frame_id: str
        :param heading: Your desired heading.
        :type heading: int
        """
        if heading is None:
            if self._setpoint_heading is not None:
                heading = self._setpoint_heading
            else:
                heading = self.get_heading()
        msg = to_pose_stamped(value, frame_id, heading)

        # transform to map frame
        msg = self.transform_pose(msg, "map", 1.0)
        if msg is not None:
            self._setpoint_pos = msg
            self._setpoint_mode = UAV.SetpointMode.POSITION

    def transform_pose(self, pose, target, timeout=1.0):
        # type: (PoseStamped, str, float) -> PoseStamped | None
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                pose.header.frame_id,
                rospy.Time.now(),
                rospy.Duration.from_sec(timeout)
            )
            pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Transforming setpoint failed: " + str(e))
            return None
        return pose

    def get_velocity(self):
        return self.autopilot.local_twist

    def set_velocity(self, value, frame_id=None):
        if isinstance(value, TwistStamped):
            msg = value
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
        if frame_id is not None:
            msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        self._setpoint_vel = msg
        self._setpoint_mode = UAV.SetpointMode.VELOCITY

    def distance_to_target(self, target=None, current=None, frame_id=None):
        d = to_numpy(self.displacement_from_target(target, current, frame_id))
        return np.linalg.norm(d)

    def displacement_from_target(self, target=None, current=None, frame_id=None):
        if target is None:
            target = self._setpoint_pos
        if current is None:
            current = self.get_pose()

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
        # Should probably allow Pose or PoseStamped objects
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
        self.home = self.get_pose()

    def hover(self):
        self.set_position(self.get_pose())

    def takeoff(self, speed=0.5, target_height=10):
        # type: (float, float) -> None
        rospy.loginfo("Waiting for arm.")
        self.wait_for(lambda: self.is_armed)
        self.set_velocity([0, 0, 0])
        rospy.loginfo("Waiting for offboard.")
        self.wait_for(lambda: self.is_offboard)
        rospy.loginfo("Takeoff!")
        self._vertical_pid(lambda: self.autopilot.relative_altitude < target_height, speed=abs(speed))
        self.hover()

    def land(self, speed=0.5, target=None):
        # type: (float, PoseStamped) -> None
        self.hover()
        position = self.get_position()
        rospy.sleep(1)  # wait to stabilize
        position[2] = 5  # 5m above ground
        self.set_position(position)
        self._vertical_pid(lambda: self.is_armed, speed=-abs(speed))


    def _vertical_pid(self, condition, p=5.0, i=0.0,  d=1.0, speed=0.0, frequency=None):
        if frequency is None:
            frequency = self.frequency
        location = conversions.to_numpy(self.get_pose())
        pid_x = PID(p, i, d)
        pid_y = PID(p, i, d)
        pid_x.SetPoint = location[0]
        pid_y.SetPoint = location[1]
        pid_x.setSampleTime(1.0 / frequency)
        pid_y.setSampleTime(1.0 / frequency)
        rate = rospy.Rate(frequency)
        while condition():
            position = conversions.to_numpy(self.get_pose())
            pid_x.update(position[0])
            pid_y.update(position[1])
            self.set_velocity([pid_x.output, pid_y.output, speed], "map")
            rate.sleep()

    def set_position(self, position, frame_id="map", heading=None, blocking=False, margin=0.5):
        """Tells multirotor to fly to maintain given position.

        :param position: The setpoint position.
        :type position: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
        :param frame_id: The frame relative to which the setpoint is set.
        :type frame_id: str
        :param heading: The heading to maintain.
        :type heading: int
        :param blocking: Weather or not to block the thread until setpoint reached.
        :type blocking: bool
        :param margin: The setpoint margin. Only matters if blocking is true.
        :type margin: float
        """
        UAV.set_position(self, position, frame_id, heading)
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()
