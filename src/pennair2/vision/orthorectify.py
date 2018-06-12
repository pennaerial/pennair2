import math
import geopy
import geopy.distance

#helper method to find one dimensional displacement
def displacement(pixel_coord, pixel_length, FOV, camera_tilt, altitude):
    target_angle = pixel_coord / pixel_length * FOV - FOV / 2 - camera_tilt;
    return altitude * math.tan(target_angle)
#print(displacement(1, 4, 0.927295,- math.pi / 4, 6))

#shift latitude or longitude by certain number of meters
def shift_coord(lat, lon, dlat, dlon):
    shift_lat = geopy.distance.VincentyDistance(kilometers = dlat)
    shift_lon = geopy.distance.VincentyDistance(kilometers = dlon)
    end_lat = shift_lat.destination(point=geopy.Point(lat, lon), bearing = 90)
    return shift_lon.destination(point=end_lat, bearing = 0)
#print(shift_coord(48.853, 2.349, 0, 1))

#find coordinates of location on a picture and returns them as a tuple
def get_coord(lat, lon, x_pixel, y_pixel, pic_width, pic_height, hor_fov, ver_fov, roll, pitch, yaw, altitude):
    """
    :param lat: latitude at time of picture taken in degrees
    :type index: float

    :param lon: longitude at time of picture taken in degrees
    :type index: float

    :param x_pixel: x coordinate of pixel of target
    :type index: int

    :param y_pixel: y coordinate of pixel of target
    :type index: int

    :param pic_width: width of picture in pixels
    :type index: int

    :param pic_height: height of picture in pixels
    :type index: int

    :param hor_fov: camera horizontal field of view in degrees
    :type index: float

    :param ver_fov: camera vertical field of view in degrees
    :type index: float

    :param roll: roll in radians
    :type index: float

    :param roll: roll in radians
    :type index: float

    :param pitch: pitch in radians
    :type index: float

    :param yaw: yaw in radians
    :type index: float

    :param altitude: altitude in kilometers
    :type index: float
    """
    dx = displacement(x_pixel, pic_width, hor_fov, roll, altitude)
    dy = displacement(y_pixel, pic_height, ver_fov, -pitch, altitude)
    dlat = dy * math.sin(yaw) + dx * math.cos(yaw)
    dlon = dy * math.cos(yaw) + dx * math.sin(yaw)
    tgt_coord = shift_coord(lat, lon, dlat, dlon)
    return (tgt_coord.latitude, tgt_coord.longitude)
#print(get_coord(48.853, 2.349, 5, 100, 100, 200, math.pi / 2, math.pi / 3, 0, 0, 180, 10))
