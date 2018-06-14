import math
from geometry_msgs.msg import Pose
import numpy as np
from tf import transformations

#helper method to find one dimensional displacement
def displacement(pixel_coord, pixel_length, FOV, camera_tilt, altitude):
    target_angle = pixel_coord / pixel_length * FOV - FOV / 2 + camera_tilt;
    return altitude * math.tan(target_angle)
#print(displacement(1, 4, 0.927295,- math.pi / 4, 6))

def qv_mult(q1, v1):
    v1 = transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return transformations.quaternion_multiply(
        transformations.quaternion_multiply(q1, q2),
        transformations.quaternion_conjugate(q1))[:3]

def relative_displacement(pose, x_pixel, y_pixel, img_width, img_height, hor_fox, ver_fov):
    # type: (Pose) -> np.ndarray
    direction = [0, 0, -1]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    direction = qv_mult(q, direction)

#find coordinates of location on a picture and returns them as a tuple
def get_coord(x_drone, y_drone, x_pixel, y_pixel, pic_width, pic_height, hor_fov, ver_fov, roll, pitch, yaw, altitude):
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
    dx = -displacement(y_pixel, pic_height, ver_fov, -pitch, altitude)
    dy = displacement(x_pixel, pic_width, hor_fov, roll, altitude)
    return (x_drone + dx, y_drone + dy)
print(get_coord(48.853, 2.349, 5, 100, 100, 200, math.pi / 2, math.pi / 3, 0, 0, 180, 10))
