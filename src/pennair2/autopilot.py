import rospy
from abc import ABCMeta
from copy import deepcopy

# ROS message imports
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import State, BatteryStatus
from mavros_msgs.srv import CommandLong, CommandInt, CommandLongRequest, CommandIntRequest


class Autopilot:
    __metaclass__ = ABCMeta

    def __init__(self):
        # region Private Fields
        self._global_global = None  # type: NavSatFix
        self._global_local = None  # type: PoseStamped
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
        :rtype: PoseStamped
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

    @property
    def local_twist(self):
        """
        Fused linear and angular velocity from the FCU.
        :return: velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._local_twist)


class Mavros(Autopilot):
    def __init__(self, mavros_prefix="/mavros"):
        """

        :param mavros_prefix: The mavros prefix.
        :type mavros_prefix: str
        """
        Autopilot.__init__(self)

        # region Private Fields
        self._state = None  # type: State
        self._battery = None  # type: BatteryStatus

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
            self._raw_gps_twist = msgint()

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
        rospy.Subscriber(mavros_prefix + "/global_position/local", PoseWithCovarianceStamped, global_local_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/gp_vel", TwistStamped, global_vel_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/rel_alt", Float64, global_rel_alt_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/compass_hdg", Float64, global_heading_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/raw/fix", NavSatFix, raw_gps_callback)
        rospy.Subscriber(mavros_prefix + "/global_position/raw/gps", TwistStamped, raw_gps_twist_callback)

        rospy.Subscriber(mavros_prefix + "/imu/data", Imu, imu_data_callback)
        rospy.Subscriber(mavros_prefix + "/imu/data_raw", Imu, imu_data_raw_callback)

        rospy.Subscriber(mavros_prefix + "/local_position/pose", PoseStamped, local_pose_callback)
        rospy.Subscriber(mavros_prefix + "/local_position/velocity", TwistStamped, local_twist_callback)

        rospy.Subscriber(mavros_prefix + "/state", State, state_callback)
        rospy.Subscriber(mavros_prefix + "/battery", BatteryStatus, battery_callback)
        # endregion

        # region Publishers
        self.acceleration_pub = rospy.Publisher(mavros_prefix + "/setpoint_accel/accel", Vector3Stamped, queue_size=1)
        self.ang_velocity_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/cmd_vel", TwistStamped, queue_size=1)
        self.attitude_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/attitude", PoseStamped, queue_size=1)
        self.throttle_pub = rospy.Publisher(mavros_prefix + "/setpoint_attitude/att_throttle", Float64, queue_size=1)
        self.position_pub = rospy.Publisher(mavros_prefix + "/setpoint_position/local", PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher(mavros_prefix + "/setpoitn_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.command_long_srv = rospy.ServiceProxy(mavros_prefix + "/cmd/Command", CommandLong)
        self.command_int_srv = rospy.ServiceProxy(mavros_prefix + "/cmd/CommandInt", CommandInt)
        # endregion

    @property
    def state(self):
        """
        The state of the autopilot. Includes things like current mode.
        :return: Mavros state
        :rtype: State
        """
        return deepcopy(self.state)

    @property
    def battery(self):
        """
        The status of the battery. This has to be properly configured in QGroundControl to be reliable.
        :return: Battery status
        :rtype: BatteryStatus
        """
        return deepcopy(self.battery)

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
            setattr(request, "param" + str(i+1), params[i])

        try:
            rsp = self.command_long_srv(request)
            return rsp.success, rsp.result
        except rospy.ServiceException as e:
            return False, None



