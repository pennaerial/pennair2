import rospy
from abc import ABCMeta, abstractmethod
from copy import deepcopy

# ROS message imports
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import WaypointList, State, BatteryStatus


class Autopilot(metaclass=ABCMeta):

    def __init__(self):

        # region Private Fields
        self._global_global: NavSatFix = None
        self._global_local: PoseStamped = None
        self._relative_altitude: float = None
        self._heading: float = None
        self._global_vel_raw: TwistStamped = None
        self._gps_raw: NavSatFix = None
        self._gps_twist_raw: TwistStamped = None

        self._imu_data: Imu = None
        self._imu_data_raw: Imu = None

        self._local_pose: PoseStamped = None
        self._local_twist: TwistStamped = None
        # endregion

    # global_position
    @property
    def global_global(self) -> NavSatFix:
        """
        The global position as a GPS coordinate.
        :return: GPS position.
        :rtype: NavSatFix
        """
        return deepcopy(self._global_global)

    @property
    def global_local(self) -> PoseStamped:
        """
        The global position in UTM coordinates.
        UTM coordinates are used to define position on Earth using a Euclidean coordinate system.
        :return: Global position in local frame
        :rtype: PoseStamped
        """
        return deepcopy(self._global_local)

    @property
    def global_velocity_raw(self) -> TwistStamped:
        """
        The raw GPS velocity.
        :return: GPS velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._global_vel_raw)

    @property
    def relative_altitude(self) -> float:
        return self._relative_altitude

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def gps_raw(self) -> NavSatFix:
        """
        Raw GPS position data.
        :return: raw GPS data
        :rtype: NavSatFix
        """
        return deepcopy(self._gps_raw)

    @property
    def gps_twist_raw(self) -> TwistStamped:
        """
        Raw GPS velocity data.
        :return: raw GPS velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._gps_twist_raw)

    # imu_pub
    @property
    def imu_data(self) -> Imu:
        """
        :return: Imu data, orientation computed by FCU
        :rtype: Imu 
        """
        return deepcopy(self._imu_data)

    @property
    def imu_raw(self) -> Imu:
        """
        :return: Raw IMU data without orientation
        :rtype: Imu
        """
        return deepcopy(self._imu_data_raw)

    # local_position
    @property
    def local_pose(self) -> PoseStamped:
        """
        Fused position in local reference frame, as defined in ~local_position/frame_id
        :return: Local position
        :rtype: PoseStamped
        """
        return deepcopy(self._local_pose)

    @property
    def local_twist(self) -> TwistStamped:
        """
        Fused linear and angular velocity from the FCU.
        :return: velocity
        :rtype: TwistStamped
        """
        return deepcopy(self._local_twist)


class Mavros(Autopilot):
    def __init__(self, mavros_prefix: str = "/mavros/"):
        super().__init__()

        if not mavros_prefix.endswith("/"):
            mavros_prefix += "/"

        # region Private Fields
        self._state = None
        self._battery = None
        # endregion

        # region Subscriber Callbacks

        def global_global_callback(msg: NavSatFix):
            self._global_global = msg

        def global_local_callback(msg: PoseStamped):
            self._global_local = msg

        def global_vel_callback(msg: TwistStamped):
            self._global_vel_raw = msg

        def global_rel_alt_callback(msg: Float64):
            self.relative_altitude = msg.data

        def global_heading_callback(msg: Float64):
            self.heading = msg.data

        def raw_gps_callback(msg: NavSatFix):
            self._raw_gps = msg

        def raw_gps_twist_callback(msg: TwistStamped):
            self._raw_gps_twist = msg

        def imu_data_callback(msg: Imu):
            self._imu_data = msg

        def imu_data_raw_callback(msg: Imu):
            self._imu_data_raw = msg

        def local_pose_callback(msg: PoseStamped):
            self._local_pose = msg

        def local_twist_callback(msg: TwistStamped):
            self._local_twist = msg

        def state_callback(msg: State):
            self.state = msg

        def battery_callback(msg: BatteryStatus):
            self.battery = msg

        def waypoints_callback(msg: WaypointList):
            self._waypoints = msg

        # endregion

        # region Subscribers
        rospy.Subscriber(mavros_prefix + "global_position/global", NavSatFix, global_global_callback)
        rospy.Subscriber(mavros_prefix + "global_position/local", PoseWithCovarianceStamped, global_local_callback)
        rospy.Subscriber(mavros_prefix + "global_position/gp_vel", TwistStamped, global_vel_callback)
        rospy.Subscriber(mavros_prefix + "global_position/rel_alt", Float64, global_rel_alt_callback)
        rospy.Subscriber(mavros_prefix + "global_position/compass_hdg", Float64, global_heading_callback)
        rospy.Subscriber(mavros_prefix + "global_position/raw/fix", NavSatFix, raw_gps_callback)
        rospy.Subscriber(mavros_prefix + "global_position/raw/gps", TwistStamped, raw_gps_twist_callback)

        rospy.Subscriber(mavros_prefix + "imu/data", Imu, imu_data_callback)
        rospy.Subscriber(mavros_prefix + "imu/data_raw", Imu, imu_data_raw_callback)

        rospy.Subscriber(mavros_prefix + "local_position/pose", PoseStamped, local_pose_callback)
        rospy.Subscriber(mavros_prefix + "local_position/velocity", TwistStamped, local_twist_callback)

        rospy.Subscriber(mavros_prefix + "state", State, state_callback)
        rospy.Subscriber(mavros_prefix + "battery", BatteryStatus, battery_callback)
        # endregion

        # region Publishers
        self.acceleration_pub = rospy.Publisher(mavros_prefix + "setpoint_accel/accel", Vector3Stamped, queue_size=1)
        self.ang_velocity_pub = rospy.Publisher(mavros_prefix + "setpoint_attitude/cmd_vel", TwistStamped, queue_size=1)
        self.attitude_pub = rospy.Publisher(mavros_prefix + "setpoint_attitude/attitude", PoseStamped, queue_size=1)
        self.throttle_pub = rospy.Publisher(mavros_prefix + "setpoint_attitude/att_throttle", Float64, queue_size=1)
        self.position_pub = rospy.Publisher(mavros_prefix + "setpoint_position/local", PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher(mavros_prefix + "setpoitn_velocity/cmd_vel", TwistStamped, queue_size=1)
        # endregion

    @property
    def state(self) -> State:
        """
        The state of the autopilot. Includes things like current mode.
        :return: Mavros state
        :rtype: State
        """
        return deepcopy(self.state)

    @property
    def battery(self) -> BatteryStatus:
        """
        The status of the battery. This has to be properly configured in QGroundControl to be reliable.
        :return: Battery status
        :rtype: BatteryStatus
        """
        return deepcopy(self.battery)
