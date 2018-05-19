# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import rospy
from launch import launch, Node
from launch import LaunchFile


class LocalizationNode(Node):
    def __init__(self, name, ukf=False, map_is_world=True, frame_prefix="", publish_tf=True, **params):
        # type: (str, bool, bool, str, bool) -> None
        Node.__init__(self, name, params)

        if ukf:
            type = "ukf_localization_node"
        else:
            type = "ekf_localization_node"

        self.add_attribute("pkg", "robot_localization")
        self.add_attribute("name", name)
        self.add_attribute("type", type)

        self.add_remap("/odometry/filtered", "/{0}/odometry/filtered".format(name))

        if map_is_world:
            self.add_param("world_frame", frame_prefix + "map")
        else:
            self.add_param("world_frame", frame_prefix + "odom")

        self.add_param("map_frame", frame_prefix + "map")
        self.add_param("odom_frame", frame_prefix + "odom")
        self.add_param("base_link_frame", frame_prefix + "fcu")

        if publish_tf:
            self.add_param("publish_tf", "true")
        else:
            self.add_param("publish_tf", "false")

    def add_source(self, name, topic, matrix):
        # type: (str, str, list[bool]) -> None
        self.add_rosparam(name, topic)
        self.add_rosparam(name + "_config", str(matrix).lower())

    def add_mavros(self, mavros_prefix="", gps=True):
        # type: (str, bool) -> None
        imu_matrix = \
            [False, False, False,
             True, True, True,
             False, False, False,
             True, True, True,
             True, True, True]
        self.add_source("imu0", mavros_prefix+"/imu/data", imu_matrix)

        if gps:
            odom_matrix = \
                [True, True, True,
                 False, False, False,
                 False, False, False,
                 False, False, False,
                 False, False, False]
            self.add_source("odom0", mavros_prefix + "/global_position/local", odom_matrix)

            gps_matrix = \
                [False, False, False,
                 True, True, True,
                 False, False, False,
                 True, True, True,
                 True, True, True]
            self.add_source("gps0", mavros_prefix + "/global_position/data", gps_matrix)


class NavstatTransformNode(Node):
    def __init__(self, name, localization_node_odometry, mavros_prefix=None, broadcast_transform=True, **params):
        # type: (str, LocalizationNode, str, bool) -> None
        Node.__init__(self, name, params)
        self.add_attribute("pkg", "robot_localization")
        self.add_attribute("name", name)
        self.add_attribute("type", "navsat_transform_node")
        self.add_param("broadcast_utm_transform", str(broadcast_transform).lower())
        self.add_remap("/odometry/filtered", '/{0}/odometry/filtered'.format(localization_node_odometry.name))
        if mavros_prefix is not None:
            self.specify_data(mavros_prefix + "/imu/data", mavros_prefix + "/global_position/raw/fix")

    def specify_data(self, imu_topic, gps_topic):
        # type: (str, str) -> None
        self.add_remap("/imu/data", imu_topic)
        self.add_remap("/gps/fix", gps_topic)