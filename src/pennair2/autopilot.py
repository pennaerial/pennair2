# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

from types import NoneType
import rospy
from abc import ABCMeta, abstractmethod
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from std_msgs.msg import Float64
from mavros_msgs.msg import State, WaypointList, Waypoint, WaypointClear
from mavros_msgs.srv import CommandLong, CommandInt, CommandLongRequest, CommandIntRequest, SetMode, CommandBool
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
from nav_msgs.msg import Odometry
from pennair2.launch import launch


class Autopilot:
    __metaclass__ = ABCMeta

    def __init__(self):
        # region Private Fields
        self._global_global = None  # type: NavSatFix
        self._global_local = None  # type: Odometry
        self._relative_altitude = None  # type: float
        self._heading = None  # type: float
        self._global_vel_raw = None  # type: TwistStamped
        self._gps_raw = None  # type: NavSatFix
        self._gps_twist_raw = None  # type: TwistStamped

        self._imu_data = None  # type: Imu
        self._imu_data_raw = None  # type: Imu

        self._local_pose = None  # type: PoseStamped
        self._local_twist = None  # type: TwistStamped
        # endregion

    def variables_initialized(self):
        # TODO: make sure null checks are done for all variables
        return (self._global_global is not None and
                self._global_local is not None and
                self._relative_altitude is not None and
                self._heading is not None and
                # self._global_vel_raw is not None and
                # self._gps_raw is not None and
                # self._gps_twist_raw is not None and
                self._imu_data is not None and
                self._imu_data_raw is not None and
                self._local_pose is not None
                # self._local_twist is not None
                )

    # global_positionW
    @property
    def global_global(self):
        """
        The global position as a GPS coordinate.
        :return: GPS position.
        :rtype: NavSatFix
        """
        return deepcopy(self._global_global)

    @property
    def global_local(self):
        """
        The global position in UTM coordinates.
        UTM coordinates are used to define position on Earth using a Euclidean coordinate system.
        :return: Global position in local frame
        :rtype: PoseWithCovarianceStamped
        """
        return deepcopy(self._global_local)

    @property
    def global_velocity_raw(self):
        """
        The raw GPS velocity.
        :return: GPS velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._global_vel_raw)

    @property
    def relative_altitude(self):
        """
        :return: The relative altitude to home.
        :rtype: float
        """
        return self._relative_altitude

    @property
    def heading(self):
        """
        :return: The heading.
        :rtype: float
        """
        return self._heading

    @property
    def gps_raw(self):
        """
        Raw GPS position data.
        :return: raw GPS data
        :rtype: NavSatFix
        """
        return deepcopy(self._gps_raw)

    @property
    def gps_twist_raw(self):
        """
        Raw GPS velocity data.
        :return: raw GPS velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._gps_twist_raw)

    # imu_pub
    @property
    def imu_data(self):
        """
        :return: Imu data, orientation computed by FCU
        :rtype: Imu
        """
        return deepcopy(self._imu_data)

    @property
    def imu_raw(self):
        """
        :return: Raw IMU data without orientation
        :rtype: Imu
        """
        return deepcopy(self._imu_data_raw)

    # local_position
    @property
    def local_pose(self):
        """
        Fused position in local reference frame, as defined in ~local_position/frame_id
        :return: Local position
        :rtype: PoseStamped
        """
        return deepcopy(self._local_pose)

    @local_pose.setter
    def local_pose(self, val):
        self._local_pose_setter(val)

    @abstractmethod
    def _local_pose_setter(self, val):
        pass

    @property
    def local_twist(self):
        """
        Fused linear and angular velocity from the FCU.
        :return: velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._local_twist)

    @local_twist.setter
    def local_twist(self, val):
        self._local_twist_setter(val)

    @abstractmethod
    def _local_twist_setter(self, val):
        pass

    @abstractmethod
    def is_connected(self):
        pass


class Mavros(Autopilot):
    def __init__(self, mavros_prefix="/mavros"):
        """

        :param mavros_prefix: The mavros prefix.
        :type mavros_prefix: str
        """
        Autopilot.__init__(self)

        # region Private Fields
        self._state = None  # type: State
        self._battery = None  # type: BatteryState
        self.mavros_prefix = mavros_prefix  # type: str

        # endregion

        # region Subscriber Callbacks

        def global_global_callback(msg):
            self._global_global = msg

        def global_local_callback(msg):
            self._global_local = msg

        def global_vel_callback(msg):
            self._global_vel_raw = msg

        def global_rel_alt_callback(msg):
            self._relative_altitude = msg.data

        def global_heading_callback(msg):
            self._heading = msg.data

        def raw_gps_callback(msg):
            self._raw_gps = msg

        def raw_gps_twist_callback(msg):
            self._raw_gps_twist = msg

        def imu_data_callback(msg):
            self._imu_data = msg

        def imu_data_raw_callback(msg):
            self._imu_data_raw = msg

        def local_pose_callback(msg):
            self._local_pose = msg

        def local_twist_callback(msg):
            self._local_twist = msg

        def state_callback(msg):
            self._state = msg

        def battery_callback(msg):
            self._battery = msg

        def waypoints_callback(msg):
            self._waypoints = msg

        # endregion

        # region Subscribers
        rospy.Subscriber(mavros_prefix + "/global_position/global", NavSatFix, global_global_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/local", Odometry, global_local_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/gp_vel", TwistStamped, global_vel_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/rel_alt", Float64, global_rel_alt_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/compass_hdg", Float64, global_heading_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/raw/fix", NavSatFix, raw_gps_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/raw/gps", TwistStamped, raw_gps_twist_callback)

        rospy.Subscriber(mavros_prefix + "/imu/data", Imu, imu_data_callback)
        rospy.Subscriber(mavros_prefix + "/imu/data_raw", Imu, imu_data_raw_callback)

        rospy.Subscriber(mavros_prefix + "/local_position/pose", PoseStamped, local_pose_callback)
        rospy.Subscriber(mavros_prefix + "/local_position/velocity", TwistStamped, local_twist_callback)

        rospy.Subscriber(mavros_prefix + "/mission/waypoints", WaypointList, waypoints_callback)

        rospy.Subscriber(mavros_prefix + "/state", State, state_callback)
        rospy.Subscriber(mavros_prefix + "/battery", BatteryState, battery_callback)
        # endregion

        # region Publishers
        # attitude setpoints
        self.ang_velocity_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/cmd_vel", TwistStamped,
                                                queue_size=1)
        self.attitude_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/attitude", PoseStamped, queue_size=1)
        self.throttle_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/att_throttle", Float64, queue_size=1)

        self.position_pub = rospy.Publisher(mavros_prefix + "/setpoint_position/local", PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher(mavros_prefix + "/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        self.acceleration_pub = rospy.Publisher(mavros_prefix + "/setpoint_accel/accel", Vector3Stamped, queue_size=1)

        self.command_long_srv = rospy.ServiceProxy(mavros_prefix + "/cmd/Command", CommandLong)
        self.command_int_srv = rospy.ServiceProxy(mavros_prefix + "/cmd/CommandInt", CommandInt)

        self.waypoints_clear = rospy.ServiceProxy(mavros_prefix + "/mission/clear", WaypointClear)
        self.waypoints_srv = rospy.ServiceProxy(mavros_prefix + "/mission/push", WaypointList)
        # endregion

    def is_connected(self):
        return (self.state is not None) and self.state.connected

    @property
    def state(self):
        """
        The state of the autopilot. Includes things like current mode.
        :return: Mavros state
        :rtype: State
        """
        return deepcopy(self._state)

    @property
    def battery(self):
        """
        The status of the battery. This has to be properly configured in QGroundControl to be reliable.
        :return: Battery status
        :rtype: BatteryState
        """
        return deepcopy(self._battery)

    def send_command_long(self, cmd, params):
        """
        Send a raw MAVLINK command.
        :param cmd: The command number.
        :type cmd: int
        :param params: A list of parameters to send in the command.
        :type params: int list
        :return: A boolean that specifies success and an integer specifying the result.
        :rtype: (bool, int)
        """
        request = CommandLongRequest()
        request.command = cmd

        for i in range(min(len(params), 7)):
            setattr(request, "param" + str(i + 1), params[i])

        try:
            rsp = self.command_long_srv(request)
            return rsp.success, rsp.result
        except rospy.ServiceException as e:
            return False, None

    def _local_pose_setter(self, val):
        # type: (PoseStamped) -> None
        msg = PoseStamped()
        msg.header.frame_id = val.header.frame_id
        msg.pose = val.pose
        self.position_pub.publish(msg)

    def _local_twist_setter(self, val):
        # type: (TwistStamped) -> None
        msg = TwistStamped()
        msg.header.frame_id = val.header.frame_id
        msg.twist = val.twist
        self.velocity_pub.publish(msg)

    def set_mission_path(self, val):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            success = self.waypoints_srv(waypoints=val)
        except rospy.ServiceException as e:
            print("failed to update waypoint table")

    def clear_mission_path(self):
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            success = self.waypoints_clear()
        except rospy.ServiceException as e:
            print("failed to clear waypoint table")

    #vtol takeoff and landing cmds (vtol takeoff: 84, vtol land: 85)
    def takeoff(self, long, lat, alt):
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            successful = takeoffService(command=84, param2=1, param4=None, param5=long, param6=lat, param7=alt)
        except rospy.ServiceException as e:
            print("failed to send takeoff cmd" % e)

    def landing(self, long, lat, alt):
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            successful = takeoffService(command=85, param1=0, param3=None, param4=None, param5=long, param6=lat, param7=None)
        except rospy.ServiceException as e:
            print("failed to send takeoff cmd" % e)

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_mission_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='AUTO.MISSION')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. AUTO.MISSION Mode could not be set." % e)

    def set_arm(self, val):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(val)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def launch_node(self, fcu_url="udp://:14540@127.0.0.1:14557", gcs_url="udp://@10.42.0.1", **kwargs):
        """
        Launch a mavros node corresponding to the appropriate
        :param fcu_url: The url of the flight control unit.
        :type fcu_url: str
        :param gcs_url: The url of the ground station (qGroundControl) for passthrough.
        :type gcs_url: str
        :param kwargs: Any other remap arguments.

        """
        pass
        # TODO: make work after update to ROS Lunar
        # launch("pennair2", "mavros.launch", name=self.mavros_prefix, fcu_url=fcu_url, gcs_url=gcs_url, **kwargs)
