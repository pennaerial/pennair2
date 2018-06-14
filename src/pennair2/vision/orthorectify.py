import math
from geometry_msgs.msg import Pose
import numpy as np
from tf import transformations

def qv_mult(q1, v1):
    v1 = transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return transformations.quaternion_multiply(
        transformations.quaternion_multiply(q1, q2),
        transformations.quaternion_conjugate(q1))[:3]

def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):

    ndotu = planeNormal.dot(rayDirection)
    if abs(ndotu) < epsilon:
        raise RuntimeError("no intersection or line is within plane")

    w = rayPoint - planePoint
    si = -planeNormal.dot(w) / ndotu
    Psi = w + si * rayDirection + planePoint
    return Psi

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
    pitch = x_pixel/pic_width * hor_fov - hor_fov / 2
    roll = y_pixel / pic_height * ver_fov - ver_fov / 2
    q1 = transformations.quaternion_from_euler(roll, pitch, 0)
    q2 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    q_total = transformations.quaternion_multiply(q2, q1)
    down = [0, 0, -1]
    direction = qv_mult(q_total, direction)
    plane_normal = [0, 0, 1]
    plane_point = [0, 0, 0]
    ray_point = [pose.position.x, pose.position.y, pose.position.z]
    (x, y, z) = LinePlaneCollision(plane_normal, plane_point, direction, ray_point)
    return x, y, z
