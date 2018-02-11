from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
import numpy as np

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