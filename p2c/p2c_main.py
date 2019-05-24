import numpy as np
import numpy.ctypeslib as npct
from ctypes import *
import ctypes as ct
# import get_point as gp
# import random
# import time

libc = npct.load_library('libmyfunc', "/home/nvidia/catkin_ws/src/team107/scripts/p2c")

rows = 160
cols = 320
array_2d_int = npct.ndpointer(dtype=np.int32, ndim=2, flags='CONTIGUOUS')
libc.get_point.argtypes = [array_2d_int, c_float, POINTER(c_int), POINTER(c_int), c_int, c_int, c_int, c_bool, c_int]
libc.check_future_road.argtypes = [array_2d_int, c_float, POINTER(c_bool), POINTER(c_int), c_int]
libc.get_center_point.argtypes = [array_2d_int, c_float, c_float, c_int, POINTER(c_int), POINTER(c_int), POINTER(c_bool), POINTER(c_bool), c_int, c_int, c_int]
libc.get_center_point_left_and_right.argtypes = [array_2d_int, c_float, c_float, c_int, POINTER(c_int), POINTER(c_int), c_int, c_int, c_float, c_int]
libc.set_time_threshold.argtypes = [c_float]

def get_point(img, roi, flag, left_restriction, right_restriction, has_road, road_property):
    # libc.get_point(a, 0.6, p_x, p_y, 0, 0, 319)
    # x = c_int(0)
    # y = c_int(0)
    p_x = pointer(c_int(0))
    p_y = pointer(c_int(0))
    libc.get_point(img, roi, p_x, p_y, flag, left_restriction, right_restriction, has_road, road_property)
    return p_y.contents.value, p_x.contents.value

def check_future_road(img, roi, margin):
    p_has_road = pointer(c_bool())
    p_road_property = pointer(c_int(0))
    libc.check_future_road(img, roi, p_has_road, p_road_property, margin)
    return p_road_property.contents.value, p_has_road.contents.value

def get_center_point(img, roi, future_roi, flag, left_restriction, right_restriction, mode):
    p_x = pointer(c_int())
    p_y = pointer(c_int())
    p_is_crossroad = pointer(c_bool())
    p_is_crossroad_control = pointer(c_bool())
    libc.get_center_point(img, roi, future_roi, flag, p_x, p_y, p_is_crossroad, p_is_crossroad_control, left_restriction, right_restriction, mode)
    return p_y.contents.value, p_x.contents.value, p_is_crossroad.contents.value, p_is_crossroad_control.contents.value

def get_center_point_left_and_right(img, roi, future_roi, flag, left_restriction, right_restriction, total_time, mode):
    p_x = pointer(c_int())
    p_y = pointer(c_int())
    libc.get_center_point_left_and_right(img, roi, future_roi, flag, p_x, p_y, left_restriction, right_restriction, total_time, mode)
    return p_y.contents.value, p_x.contents.value
