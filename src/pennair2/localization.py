# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import rospy
from launch import launch
from launch import LaunchFile


class Generator:
    def __init__(self):
        self.launch = LaunchFile()
        self.node = None

    def add_localization(self, name, mavros_prefix=None, is_ukf=True, frame_prefix="", map_is_world=True,
                         publish_tf=True, **params):
        localization = LaunchFile.Node(name, params)  # type: LaunchFile.Node
        self.launch.add_node(name, localization)

        type = None  # type: str
        if is_ukf:
            type = "ukf_localization_node"
        else:
            type = "ekf_localization_node"

        localization.element.set("pkg", "robot_localization")
        localization.element.set("name", name)
        localization.element.set("type", type)

        if map_is_world:
            localization.add_param("world_frame", frame_prefix + "map")

        else:
            localization.add_param("world_frame", frame_prefix + "odom")

        localization.add_param("map_frame", frame_prefix + "map")
        localization.add_param("odom_frame", frame_prefix + "odom")
        localization.add_param("baselink_frame", frame_prefix + "fcu_local")

        if publish_tf:
            localization.add_param("publish_tf", "true")
        else:
            localization.add_param("publish_tf", "false")

        for key, value in params.iteritems():
            localization.add_param(key, value)

        if mavros_prefix is not None:
            localization.add_param("imu0", mavros_prefix + "/imu/data")
            imu_matrix = \
                [False, False, False,
                 True, True, True,
                 False, False, False,
                 True, True, True,
                 True, True, True]
            localization.add_rosparam("imu0_config", str(imu_matrix).lower())

            localization.add_param("odom0", mavros_prefix + "/global_position/local")
            odom_matrix = \
                [True, True, True,
                 False, False, False,
                 False, False, False,
                 False, False, False,
                 False, False, False]
            localization.add_rosparam("odom0_config", str(odom_matrix).lower())

            localization.add_param("gps0", mavros_prefix + "/global_position/global")
            gps_matrix = \
                [False, False, False,
                 True, True, True,
                 False, False, False,
                 True, True, True,
                 True, True, True]
            localization.add_rosparam("gps0_config", str(gps_matrix).lower())

    def add_navstat_transform(self, name, mavros_prefix=None, broadcast_transform=True, **params):
        transform = LaunchFile.Node(name,params)
        self.launch.add_node(name, transform)
        transform.element.set("pkg", "robot_localization")
        transform.element.set("name", name)
        transform.element.set("type", "navsat_transform_node")

        transform.add_param("broadcast_utm_transform", str(broadcast_transform).lower())
        for key, value in params.iteritems():
            transform.add_param(key, value)

        if mavros_prefix is not None:
            transform.add_remap("/imu/data", mavros_prefix + "/imu/data")
            transform.add_remap("/gps/fix", mavros_prefix + "/gps/fix")

    def write(self, package, name):
        self.launch.write(package, name)


if __name__ == "__main__":
    rospy.init_node("node")
    generator = Generator()
    generator.add_localization("localization", mavros_prefix="/mavros")
    generator.add_navstat_transform("navstat_transform")
    generator.write("pennair2", "test.launch")
    launch("pennair2", "test.launch")