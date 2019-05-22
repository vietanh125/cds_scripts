import numpy as np
import time
import math
from collections import deque
from p2c import p2c_main
# from python_to_c import python_to_c

total_time = 0
last = time.time()
pre_flag = 0

class SegmentToSteer():
    def __init__(self, square=7, margin=30, roi=1 / 3):
        self.square = square
        self.margin = margin
        self.speed_memory = deque(iterable=np.zeros(10, dtype=np.uint8), maxlen=10)
        self.direction_queue = deque(iterable=np.zeros(3, dtype=np.uint8), maxlen=3)
        self.tilt_queue = deque(iterable=np.ones(15, dtype=np.uint8), maxlen=15)

        self.roi = 1 - roi
        self.future_roi = 0.4
        self.speed_max = 23
        self.speed_min = 18
        self.speed_brake = 10
        self.acc_threshold = 0.9
        self.is_on_bridge = False

        self.error_proportional_ = 0.0
        self.error_integral_ = 0.0
        self.error_derivative_ = 0.0
        self.k_p = 0.45
        self.k_i = 0.0001
        self.k_d = 1
        self.inc_p = 0.01
        self.inc_i = 0.0001
        self.inc_d = 0.01
        self.total_time_steer = 0.0
        self.total_time_steer_thresh = 12.0
        self.total_width = self.roi * 160
        self.counter = 1

    def pid(self, cte):
        self.error_integral_ += cte
        self.error_derivative_ = cte - self.error_proportional_
        self.error_proportional_ = cte
        return -(self.k_p * self.error_proportional_ + self.k_i * self.error_integral_ + self.k_d * self.error_derivative_)

    def check_future_road(self, img, roi, left_restriction, right_restriction):
        IMG_H, IMG_W = 160, 320
        i = int(IMG_H * roi)
        border = int((self.square - 1) / 2)
        i_l = left_restriction + border
        i_r = right_restriction - border
        has_road = False
        turn_left = False
        turn_right = False
        while i_l < right_restriction + 1 - border:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_l - border, i_l + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_l <= self.margin:
                    turn_left = True
                has_road = True
                break
            i_l += (border + 1)
        while i_r > i_l:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_r - border, i_r + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_r >= IMG_W - self.margin:
                    turn_right = True
                has_road = True
                break
            i_r -= (border + 1)
        if turn_left and not turn_right:
            return -1, has_road
        elif turn_right and not turn_left:
            return 1, has_road
        return 0, has_road


    def get_point_left_and_right(self, img, flag, s, roi, left_restriction, right_restriction):
        IMG_H, IMG_W = 160, 320
        i = int(IMG_H * roi)
        border = int((self.square - 1) / 2)
        i_l = left_restriction + border
        i_r = right_restriction - border
        turn_left = False
        turn_right = False
        # error = self.total_width / (2 * self.counter)
        error = 146
        while i_l < right_restriction + 1 - border:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_l - border, i_l + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_l <= self.margin:
                    turn_left = True
                break
            i_l += (border + 1)
        while i_r > i_l:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_r - border, i_r + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_r >= IMG_W - self.margin:
                    turn_right = True
                break
            i_r -= (border + 1)

        if turn_left and not turn_right:
            return i, max(0, min(int(i_r - error), IMG_W - 1))
        elif turn_right and not turn_left:
            return i, max(0, min(int(i_l + error), IMG_W - 1))
        # di thang
        elif turn_left and turn_right:
            if flag == 1:
                while img[i][i_r] == 1 and i >= 0:
                    i -= 1
                return i + 1, i_r
            elif flag == -1:
                while img[i][i_l] == 1 and i >= 0:
                    i -= 1
                return i + 1, i_l
        # if not turn_left and not turn_right and roi == 0.6:
        #     self.total_width += i_r - i_l
        #     self.counter += 1
        #     print self.total_width / self.counter
        return i, int((i_l + i_r) / 2)

    def get_point(self, img, flag, s, roi, left_restriction, right_restriction, has_road, road_property):
        IMG_H, IMG_W = 160, 320
        i = int(IMG_H * roi)
        border = int((self.square - 1) / 2)
        i_l = left_restriction + border
        i_r = right_restriction - border
        turn_left = False
        turn_right = False
        while i_l < right_restriction + 1 - border:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_l - border, i_l + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_l <= self.margin:
                    turn_left = True
                break
            i_l += (border + 1)
        while i_r > i_l:
            white = 0
            for m in range(i - border, i + border + 1):
                for n in range(i_r - border, i_r + border + 1):
                    white += img[m][n]
            if white == self.square ** 2:
                if i_r >= IMG_W - self.margin:
                    turn_right = True
                break
            i_r -= (border + 1)

        if (turn_left and turn_right and flag == 0):
            flag = self.get_direction()
        if (turn_left and turn_right and flag == 1) or ((turn_right and not turn_left and flag != -1) and not has_road):
            # re phai
            self.direction_queue.append(2)
            while img[i][i_r] == 1 and i >= 0:
                i -= 1
            return i + 1, i_r
        elif (turn_left and turn_right and flag == -1) or ((turn_left and not turn_right and flag != 1) and not has_road):
            # re trai
            self.direction_queue.append(0)
            while img[i][i_l] == 1 and i >= 0:
                i -= 1
            return i + 1, i_l
        self.direction_queue.append(1)
        # di thang
        return i, int((i_l + i_r) / 2)

    def append_tilt(self, acc_x, acc_y, acc_z):
        flag = 1
        pitch = (math.atan2(acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180) / math.pi
        if pitch <= -15:
            flag = 2
        elif pitch >= 15:
            flag = 0
        self.tilt_queue.append(flag)

    def get_tilt_direction(self):
        tong = 0
        for x in self.tilt_queue:
            tong += (x - 1)
        if tong == self.tilt_queue.maxlen:
            return 1
        elif tong <= self.tilt_queue.maxlen/(-5):
            return -1
        return 0

    def get_direction(self):
        arr = np.asarray(self.direction_queue, np.int8)
        return np.bincount(arr).argmax() - 1

    def mean_speed_queue(self):
        mean = 0
        for x in self.speed_memory:
            mean += x
        return mean / len(self.speed_memory)

    def get_steer(self, label, flag, s, left_restriction, right_restriction):
        global pre_flag
        # global total_time
        global last
        IMG_H, IMG_W = label.shape
        # label = label.tolist()
        label = label.astype(np.int32)
        interval = time.time() - last
        last = time.time()
        roi = self.roi
        froi = self.future_roi
        self.total_time_steer += interval
        current_flag = flag
        tilt_flag = self.get_tilt_direction()
        #-------------------------------comment for changes, may comeback later
        if tilt_flag == 1 and not self.is_on_bridge:
            self.is_on_bridge = True
        elif tilt_flag == 0 and self.is_on_bridge and pre_flag == -1:
            self.is_on_bridge = False

        pre_flag = tilt_flag
        if self.is_on_bridge:
            roi = 0.85
            froi = 0.85
        #----------------------------------------------
        # if tilt_flag == 1 or tilt_flag == -1:
        #     roi = 0.9
        #     froi = 0.9
        print roi, froi
        # if total_time > 0 and total_time < 3.5:
        #     if flag != pre_flag and flag != 0:
        #         total_time = 0
        #         pre_flag = flag
        #     total_time += interval
        #     current_flag = pre_flag
        # else:
        #     total_time = 0
        #     current_flag = flag
        #     if current_flag != 0:
        #         pre_flag = current_flag
        #         total_time += interval
        #     else:
        #         pre_flag = current_flag
        # old steer code
        # road_property, has_road = self.check_future_road(label, 0.4, 0, IMG_W - 1)
        # y, x = self.get_point_left_and_right(label, current_flag, s, roi, left_restriction, right_restriction)
        # while label[y][x] == 0 and roi < 0.9:
        #     roi += 0.05
        #     y, x = self.get_point_left_and_right(label, current_flag, s, roi, left_restriction, right_restriction)

        # new steer code
        y, x = p2c_main.get_center_point(label, roi, froi, current_flag, left_restriction, right_restriction)
        # steer = np.arctan((x - IMG_W / 2 + 1) / (IMG_H - float(y))) * 57.32
        steer = -self.pid(x - IMG_W / 2 + 1)
        steer = np.sign(steer) * min(60, abs(steer))
        #if tilt_flag == -1:
            #self.speed_memory.append(15)
        #elif tilt_flag == 1:
            #self.speed_memory.append(18)
        # if left_restriction >= 0.5 * IMG_W or right_restriction <= 0.5 * IMG_W:
        #     self.speed_memory.append(self.speed_min)
        #else:
        self.speed_memory.append((self.speed_min - self.speed_max) * steer**2/3600 + self.speed_max)
        speed = self.mean_speed_queue()
        return speed, steer
