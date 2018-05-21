# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import numpy as np

import rospy
import utm
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf import transformations


def feet_to_meters(feet):
    # One meter is 0.3048
    return 0.3048 * feet


def meters_to_feet(meters):
    return meters / 0.3048


def miles_to_meters(miles):
    # One mile is 1609.344 meters
    return 1609.344 * miles


def meters_to_miles(meters):
    return meters / 1609.344


def knots_to_mets(knots):
    return 0.514444444444 * knots


def mets_to_knots(mets):
    return 1.94384449244 * mets


def NavSatFix_to_PoseStamped(gps, force_utm_zone=None):
    # type: (NavSatFix | list[float,float,float] | (float,float,float), int) -> PoseStamped
    """
    Convert GPS to UTM position.

    :param gps: A gps position. If list/tuple given then (longitude, latitude, altitude). Altitude in WSG84.
    :param force_utm_zone: If not None, then will force utm zone as specified.
    """
    pose_stamped = PoseStamped()
    if isinstance(gps, NavSatFix):
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(gps.latitude, gps.longitude, force_utm_zone)
        pose_stamped.header = gps.header
        pose_stamped.header.frame_id = "utm"
        pose_stamped.pose.position.x = easting
        pose_stamped.pose.position.y = northing
        pose_stamped.pose.position.z = gps.altitude
    else:
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(gps[1], gps[0], force_utm_zone)
        pose_stamped.header.frame_id = "utm"
        pose_stamped.pose.position.x = easting
        pose_stamped.pose.position.y = northing
        pose_stamped.pose.position.z = gps[2]
    return pose_stamped


def gps_to_utm(latitude, longitude, altitude, force_utm_zone=None):
    # type: (float, float, float, int) -> (np.ndarray, zone_letter, zone_number)
    """
    Convert GPS to UTM position.

    :param latitude: GPS latitude.
    :param longitude: GPS longitude.
    :param altitude: Altitude in WSG84.
    :param force_utm_zone: If not None, then will force utm zone as specified.
    :return: UTM position in ENU and the zone number.
    """
    (easting, northing, zone_number, zone_letter) = utm.from_latlon(latitude, longitude, force_utm_zone)
    return np.array([[easting, northing, altitude]]).T


def to_numpy(position):
    """
    Create a numpy array (as a column vector) representation of a geometry_msgs position.
    :param position:
    :type position: PoseStamped | Pose | Point | Vector3
    """
    if isinstance(position, (Point, Vector3)):
        return np.array([[position.x, position.y, position.z]]).T
    if isinstance(position, Pose):
        return to_numpy(position.position)
    if isinstance(position, PoseStamped):
        return to_numpy(position.pose.position)


def to_pose_stamped(value, frame_id=None, heading=None):
    """ Converts a python list / tuple into a PoseStamped message. Specify frame_id and heading if necessary.

    :param value: The desired position setpoint, only yaw component of orientation is used. Can be of type
        PoseStamped, Pose, Point, or an indexable object with 3 integer elements (list, tuple, numpy array etc.)
    :type value: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
    :param frame_id: The name of the frame to use for the message. **Will not override frame_id of PoseStamped**.
    :type frame_id: str
    :param heading: Your desired heading in **radians**. This is the rotation around the **z-axis**.
    :type heading: int
    """
    if isinstance(value, PoseStamped):
        msg = value
    else:
        msg = PoseStamped()
        if isinstance(value, Pose):  # if Pose then position and orientation already provided
            msg.pose = value
        else:
            if isinstance(value, Point):
                msg.pose.position = value
            else:
                msg.pose.position.x = value[0]
                msg.pose.position.y = value[1]
                msg.pose.position.z = value[2]

            if heading is not None:
                q = transformations.quaternion_from_euler(0, 0, heading)
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]
            else:
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z = 0
                msg.pose.orientation.w = 0

    if frame_id is not None:
        if isinstance(frame_id, str):
            msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    return msg
