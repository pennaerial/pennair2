import math
from geometry_msgs.msg import Pose
import numpy as np
from tf import transformations

#print(displacement(1, 4, 0.927295,- math.pi / 4, 6))
def qv_mult(q1, v1):
    v1 = transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return transformations.quaternion_multiply(
        transformations.quaternion_multiply(q1, q2),
        transformations.quaternion_conjugate(q1))[:3]

#find coordinates of location on a picture and returns them as a tuple
def get_coord(pose, x_pixel, y_pixel, pic_width, pic_height, hor_fov, ver_fov):
    """
    :param x_drone: x-position of drone
    :type lat: float

    :param y_drone: y-position of drone
    :type lon: float

    :param x_pixel: x coordinate of pixel of target
    :type x_pixel: int

    :param y_pixel: y coordinate of pixel of target
    :type y_pixel: int

    :param pic_width: width of picture in pixels
    :type pic_width: int

    :param pic_height: height of picture in pixels
    :type pic_height: int

    :param hor_fov: camera horizontal field of view in degrees
    :type hor_fov: float

    :param ver_fov: camera vertical field of view in degrees
    :type ver_fov: float

    :param roll: roll in radians
    :type roll: float

    :param pitch: pitch in radians
    :type pitch: float

    :param yaw: yaw in radians
    :type yaw: float

    :param altitude: altitude in kilometers
    :type altitude: float
    """
    roll = x_pixel/pic_width * hor_fov - hor_fov / 2
    pitch = y_pixel / pic_height * ver_fov - ver_fov / 2
    q1 = transformations.quaternion_from_euler(roll, pitch, 0, axes='srpy')
    q2 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    q_total = transformations.quaternion_multiply(q2, q1)
    direction = [0, 0, -1]
    (dx, dy, dz) = qv_mult(q_total, direction)
    return pose.position.x + dx, pose.position.y + dy, pose.position.z
