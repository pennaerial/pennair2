# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import numpy as np
import rospy
from tf import transformations
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

GET_POSITION_TUPLE_WARN = False

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

    def get_position(self, utm=False, fmt="pose"):
        # type: (bool, str) -> PoseStamped
        if fmt is "pose":
            if utm:
                pose_covariance = self.autopilot.global_local
                return PoseStamped(pose_covariance.header, pose_covariance.pose.pose)
            else:
                return self.autopilot.local_pose
        elif fmt is "tuple":
            global GET_POSITION_TUPLE_WARN
            if not GET_POSITION_TUPLE_WARN:
                rospy.logwarn("Get position as tuple is deprecated. Use conversions.to_numpy instead.")
                GET_POSITION_TUPLE_WARN = True
            p = self.get_position(utm, fmt="pose")
            return (p.pose.position.x, p.pose.position.y, p.pose.position.z)

    def compute_pose(self,position,heading=None,frame_id='map'):
        # converts a position, heading, and frame_id into a pose
        # compute pose
        if heading is None:
            if self._setpoint_heading is not None:
                heading = self._setpoint_heading
            else:
                heading = self.get_heading()
        msg = to_pose_stamped(value, frame_id, heading)
        # transform to map frame
        msg = self.transform_pose(msg, "map", 1.0)
        return msg

    def set_position(self, position, heading=None, relative=False, frame_id="map"):
        """
        :param value: The desired position setpoint, only yaw component of orientation is used. Can be of type
            PoseStamped, Pose, Point, or an indexable object with 3 integer elements (list, tuple, numpy array etc.)
        :type value: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
        :param frame_id: The name of the frame to use for the message.
            Will only be applied if value is not already PoseStamped.
        :type frame_id: str
        :param heading: Yaw in degrees
        :type heading: int
        """
        msg = self.compute_pose(position=position,heading=heading,frame_id=frame_id)
        if msg is not None:
            if relative:
                pos = self.get_position()
                self._setpoint_pos = msg.pose.position.x + pos.pose.position.x
                self._setpoint_pos = msg.pose.position.y + pos.pose.position.y
                self._setpoint_pos = msg.pose.position.z + pos.pose.position.z
                self.set_yaw(heading,relative=True,frame_id=frame_id)
            else:
                self._setpoint_pos = msg
            self._setpoint_mode = UAV.SetpointMode.POSITION

    def cancel_setpoint(self):
        setpoint = self.get_position()
        self._setpoint_pos = msg
        self._setpoint_mode = UAV.SetpointMode.POSITION

    def set_x(self,x,relative=False,frame_id='map'):
        if self._setpoint_pos is not None:
            setpoint = self._setpoint_pos
        else:
            setpoint = self.get_position()
        if relative:
            x += self.get_position().pose.position.x
        xpose = self.compute_pose([x,0,0],frame_id=frame_id)
        if xpose is not None:
            setpint.pose.position.x = xpose.pose.position.x
            self._setpoint_pos = setpoint
            self._setpoint_mode = UAV.SetpointMode.POSITION

    def set_y(self,y,relative=False,frame_id='map'):
        if self._setpoint_pos is not None:
            setpoint = self._setpoint_pos
        else:
            setpoint = self.get_position()
        if relative:
            y += self.get_position().pose.position.y
        ypose = self.compute_pose([0,y,0],frame_id=frame_id)
        if ypose is not None:
            setpint.pose.position.y = ypose.pose.position.y
            self._setpoint_pos = setpoint
            self._setpoint_mode = UAV.SetpointMode.POSITION

    def set_z(self,z,relative=False,frame_id='map'):
        if self._setpoint_pos is not None:
            setpoint = self._setpoint_pos
        else:
            setpoint = self.get_position()
        if relative:
            z += self.get_position().pose.position.z
        zpose = self.compute_pose([0,0,z],frame_id=frame_id)
        if zpose is not None:
            setpint.pose.position.z = zpose.pose.position.z
            self._setpoint_pos = setpoint
            self._setpoint_mode = UAV.SetpointMode.POSITION

    def set_yaw(self,yaw,relative=False,frame_id='map'):
        # sets yaw in degrees
        heading = (math.pi /180.0) * yaw
        if self._setpoint_pos is not None:
            setpoint = self._setpoint_pos
        else:
            setpoint = self.get_position()
        if relative:
            heading += transformations.euler_from_quaternion(self.get_position().orientation)[2] # indexes yaw from (roll,pitch,yaw)
        hpose = self.compute_pose([0,0,0],heading=heading,frame_id=frame_id)
        if hpose is not None:
            setpint.pose.orientation = hpose.pose.orientation
            self._setpoint_pos = setpoint
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
        self.home = self.get_position()

    def hover(self):
        self.set_position(self.get_position())

    def takeoff(self, speed=1, target_height=10):
        # type: (float, float) -> None
        rospy.loginfo("Waiting for arm.")
        self.wait_for(lambda: self.is_armed)
        self.set_velocity([0, 0, 0])
        rospy.loginfo("Waiting for offboard.")
        self.wait_for(lambda: self.is_offboard)
        rospy.loginfo("Takeoff!")

        takeoff_location = conversions.to_numpy(self.get_position())
        pid_x = PID(0.5, 0.0, 0.1)
        pid_y = PID(0.5, 0.0, 0.1)
        pid_x.setSampleTime(1.0 / self.frequency)
        pid_y.setSampleTime(1.0 / self.frequency)
        rate = rospy.Rate(self.frequency)
        while self.autopilot.relative_altitude < target_height:
            position = conversions.to_numpy(self.get_position())
            error = takeoff_location - position
            pid_x.update(error[0])
            pid_y.update(error[1])
            self.set_velocity([pid_x.output, pid_y.output, abs(speed)])
            rate.sleep()
        self.hover()

    def land(self, speed=0.5, target=None):
        # type: (float, PoseStamped) -> None
        self.hover()
        rospy.sleep(1)  # wait to stabilize
        land_location = conversions.to_numpy(self.get_position())
        pid_x = PID(0.5, 0.0, 0.1)
        pid_y = PID(0.5, 0.0, 0.1)
        pid_x.setSampleTime(1.0 / self.frequency)
        pid_y.setSampleTime(1.0 / self.frequency)
        rate = rospy.Rate(self.frequency)
        while self.is_armed:
            position = conversions.to_numpy(self.get_position())
            error = land_location - position
            pid_x.update(error[0])
            pid_y.update(error[1])
            self.set_velocity([pid_x.output, pid_y.output, -abs(speed)])
            rate.sleep()

    def set_position(self, position, frame_id="map", heading=None, blocking=False, margin=0.5, relative=False):
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
        UAV.set_position(self, position=position, heading=heading, relative=relative, frame_id=frame_id)
        if not blocking:
            return
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()

    def set_x(self, x, relative=False, blocking=False, margin=0.5, frame_id="map"):
        UAV.set_x(self, x, relative=relative, frame_id=frame_id)
        if not blocking:
            return
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()

    def set_y(self, y, relative=False, blocking=False, margin=0.5, frame_id="map"):
        UAV.set_y(self, y, relative=relative, frame_id=frame_id)
        if not blocking:
            return
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()

    def set_z(self, z, relative=False, blocking=False, margin=0.5, frame_id="map"):
        UAV.set_z(self, z, relative=relative, frame_id=frame_id)
        if not blocking:
            return
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()

    def set_yaw(self, yaw, relative=False, blocking=False, margin=0.5, frame_id="map"):
        UAV.set_yaw(self, yaw, relative=relative, frame_id=frame_id)
        if not blocking:
            return
        rate = rospy.Rate(self.frequency)
        while self.distance_to_target() > margin:
            rate.sleep()