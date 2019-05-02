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
libc.check_future_road.argtypes = [array_2d_int, c_float, POINTER(c_bool), POINTER(c_int), c_int, c_int]
libc.get_center_point.argtypes = [array_2d_int, c_float, c_float, c_int, POINTER(c_int), POINTER(c_int), c_int, c_int]

def get_point(img, roi, flag, left_restriction, right_restriction, has_road, road_property):
    # libc.get_point(a, 0.6, p_x, p_y, 0, 0, 319)
    # x = c_int(0)
    # y = c_int(0)
    p_x = pointer(c_int(0))
    p_y = pointer(c_int(0))
    libc.get_point(img, roi, p_x, p_y, flag, left_restriction, right_restriction, has_road, road_property)
    return p_y.contents.value, p_x.contents.value

def check_future_road(img, roi, left_restriction, right_restriction):
    p_has_road = pointer(c_bool())
    p_road_property = pointer(c_int(0))
    libc.check_future_road(img, roi, p_has_road, p_road_property, left_restriction, right_restriction)
    return p_road_property.contents.value, p_has_road.contents.value

def get_center_point(img, roi, future_roi, flag, left_restriction, right_restriction):
    p_x = pointer(c_int())
    p_y = pointer(c_int())
    libc.get_center_point(img, roi, future_roi, flag, p_x, p_y, left_restriction, right_restriction)
    return p_y.contents.value, p_x.contents.value

# arr = np.zeros((rows, cols))
# base_roi = 0.6
# loop = 1000
# last_time = time.time()
# for index in range(loop):
#     roi = base_roi
#     list_arr = arr.tolist()
#     has_road, road_property = gp.check_future_road(list_arr, 0.4, 0, cols - 1)
#     y, x = gp.get_point(list_arr, 0, 0, roi, 0, cols - 1, has_road, road_property)
#     while list_arr[y][x] == 0 and roi < 0.9:
#         roi += 0.05
#         y_x = gp.get_point(list_arr, 0, 0, roi, 0, cols - 1, has_road, road_property)
# total_time1 = time.time() - last_time
# last_time = time.time()
# for index in range(loop):
#     roi = base_roi
#     arr_c = arr.astype(np.int32)
#     road_property, has_road = check_future_road(arr_c, 0.4, 0, cols - 1)
#     y, x = get_point(arr_c, roi, 0, 0, cols - 1, has_road, road_property)
#     while arr_c[y][x] == 0 and roi < 0.9:
#         roi += 0.05
#         y, x = get_point(arr_c, roi, 0, 0, cols - 1, has_road, road_property)
#     # y, x = get_center_point(arr_c, 0.6, 0.4, 0, 0, cols - 1)
# total_time2 = time.time() - last_time

# last_time = time.time()
# for index in range(loop):
#     arr_c = arr.astype(np.int32)
#     # road_property, has_road = check_future_road(arr_c, 0.4, 0, cols - 1)
#     # y, x = get_point(arr_c, 0.6, 0, 0, cols - 1, has_road, road_property)
#     y, x = get_center_point(arr_c, base_roi, 0.4, 0, 0, cols - 1)
# total_time3 = time.time() - last_time

# print total_time1 / total_time3
