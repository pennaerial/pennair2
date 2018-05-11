# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

import numpy as np
import utm
from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


def feet_to_meters(feet):
    # One meter is 0.3048
    return 0.3048 * feet

def meters_to_feet(meters):
    return meters / 0.3048


def miles_to_meters(miles):
    # One mile is 1609.344 meters
    return  1609.344 * miles


def meters_to_miles(meters):
    return meters / 1609.344


def knots_to_mets(knots):
    return 0.514444444444 * knots


def mets_to_knots(mets):
    return 1.94384449244 * mets

def gps_to_utm(gps, force_utm_zone=None):
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

def position_to_numpy(position):
    """
    Create a numpy array (as a column vector) representation of a geometry_msgs position.
    :param position:
    :type position: PoseStamped | Pose | Point | Vector3
    """
    if isinstance(position, (Point, Vector3)):
        return np.array([[position.x, position.y, position.z]]).T
    if isinstance(position, Pose):
        return position_to_numpy(position.position)
    if isinstance(position, PoseStamped):
        return position_to_numpy(position.pose.position)
