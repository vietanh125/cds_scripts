import cv2
import numpy as np
import time
import math
from collections import deque
total_time = 0
last = time.time()
pre_flag = 0
map = []
total_distance = 0.0
last_speed = 0.0
first_tine = True
stored_time = 0.0


class SegmentToSteer():
    def __init__(self, square=5, margin=30, roi=1/3, size=3):
        self.square = square
        self.margin = margin
        self.size = size
        self.memory = deque(iterable=np.zeros(size, dtype=np.uint8), maxlen=size)
        self.roi = 1 - roi
        self.speed_max = 15
        self.speed_min = 12
        # self.steer_pid = PID(Kp=self.p[0], Ki=self.p[1], Kd=self.p[2])
        # self.steer_pid.setWindup(50)
        self.error_proportional_ = 0.0
        self.error_integral_     = 0.0
        self.error_derivative_   = 0.0
        self.k_p = 0.5
        self.k_i = 0.00
        self.k_d = 0.00
        self.inc_p = 0.01
        self.inc_i = 0.0001
        self.inc_d = 0.01

    def pid(self, cte):
        self.error_integral_     += cte
        self.error_derivative_    = cte - self.error_proportional_
        self.error_proportional_  = cte

        return -(self.k_p * self.error_proportional_ + self.k_i * self.error_integral_ + self.k_d * self.error_derivative_)

    def get_point(self, img, flag, roi):
        IMG_H, IMG_W = img.shape
        i = int(IMG_H * roi)
        border = int((self.square - 1) / 2)
        i_l = border
        i_r = IMG_W - 1 - border
        turn_left = False
        turn_right = False
        while i_l < IMG_W - border:
            check = img[i - border: i + border + 1, i_l - border: i_l + border + 1]
            white = np.sum(check) / 255
            if white == self.square ** 2:
                if i_l <= self.margin:
                    turn_left = True
                break
            i_l += (border + 1)
        while i_r > i_l:
            check = img[i - border: i + border + 1, i_r - border: i_r + border + 1]
            white = np.sum(check) / 255
            if white == self.square ** 2:
                if i_r >= IMG_W - self.margin:
                    turn_right = True
                break
            i_r -= (border + 1)
        if (turn_left and turn_right and flag == 1) or (turn_right and not turn_left and roi > self.roi):
            while img[i][i_r] == 255 and i >= 0:
				i -= 1
            return i+1, i_r
        elif (turn_left and turn_right and flag == -1) or (turn_left and not turn_right and roi > self.roi):
            while img[i][i_l] == 255 and i >= 0:
                i -= 1
            return i+1, i_l
        else:
            return i, int((i_l + i_r) / 2)
     def get_point_2(self, img, flag):
        IMG_H, IMG_W = img.shape
        i = int(IMG_H * self.roi)
        border = int((self.square - 1) / 2)
        # turn_left = False
        # turn_right = False
        n_row = 0
        y = 0
        x = 0
        while i < (IMG_H-border):
            i_l = border
            i_r = IMG_W - 1 - border
            while i_l < IMG_W - border:
                check = img[i - border: i + border + 1, i_l - border: i_l + border + 1]
                white = np.sum(check) / 255
                if white == self.square ** 2:
                    # if i_l <= self.margin:
                    #     turn_left = True
                    break
                i_l += (border + 1)
            while i_r > i_l:
                check = img[i - border: i + border + 1, i_r - border: i_r + border + 1]
                white = np.sum(check) / 255
                if white == self.square ** 2:
                    # if i_r >= IMG_W - self.margin:
                    #     turn_right = True
                    break
                i_r -= (border + 1)
            if i_r > i_l:
                # img[i][int((i_l+i_r)/2)] = 127
                x += (i_r + i_l)/2
                y += i
                n_row += 1
            i += 1
        return int(y/n_row), int(x/n_row)
    def get_point_3(self, img, flag):
        IMG_H, IMG_W = img.shape
        count = 0
        turn_left = False
        turn_right = False
        x_left, y_left, x_right, y_right = 0, 0, 0, 0
        for i in range(int(IMG_H * self.roi), IMG_H):
            count += 1
            left = 0
            while img[i][left] != 255 and left < IMG_W/2:
                left += 1
            x_left += left
            y_left += i
            right = IMG_W-1
            while img[i][right] != 255 and right >= IMG_W/2:
                right -= 1
            x_right += right
            y_right += i
        x_left, y_left = int(x_left/count), int(y_left/count)
        x_right, y_right = int(x_right/count), int(y_right/count)
        if x_left <= self.margin:
            turn_left = True
        if x_right >= IMG_W - self.margin:
            turn_right = True
        if (turn_left and not turn_right) or (turn_left and turn_right and flag == -1):
            return y_left, x_left
        elif (turn_right and not turn_left) or (turn_left and turn_right and flag == 1):
            return y_right, x_right

    def discretize(self, steer):
        sign = np.sign(steer)
        steer = abs(steer)
        s = 0
        if steer > 40:
            s = 45
        elif 30 < steer <= 40:
            s = 35
        elif 20 < steer <= 30:
            s = 25
        elif 10 < steer <= 20:
            s = 15
        elif 5 < steer <= 10:
            s = 5
        elif 0 < steer <= 5:
            s = 0
        s *= sign
        return s
    def get_flag(self):
        arr = np.asarray(self.memory, np.int8)
        return np.bincount(arr).argmax() - 1

    def write_image(self, raw, label, x, y, steer):
        raw[label == 255] = [128, 64, 128]
        font = cv2.FONT_HERSHEY_DUPLEX
        bottomLeftCornerOfText = (180, 158)
        fontScale = 0.5
        fontColor = (0, 255, 0)
        lineType = 1
        cv2.putText(raw, str(steer.__format__(".0f")), bottomLeftCornerOfText, font, fontScale, fontColor, lineType, 0)
        cv2.line(raw, (x, y), (160, 160), fontColor, thickness=2)
        raw = cv2.resize(raw, (480, 240))
        return raw

    def write_vector(self, img, x, y, steer):
        IMG_H, IMG_W = img.shape
        font = cv2.FONT_HERSHEY_DUPLEX
        bottomLeftCornerOfText = (180, 158)
        fontScale = 0.5
        fontColor = (0, 255, 0)
        lineType = 1
        cv2.putText(img, str(steer.__format__(".0f")), bottomLeftCornerOfText, font, fontScale, fontColor, lineType, 0)
        cv2.line(img, (int (IMG_H / 2), IMG_W), (x, y), (0, 0, 0), 1)
        return img

    def get_steer(self, raw, label, flag, s):
        global pre_flag
        global total_time
        global last
        global first_tine
        global stored_time
        global last_speed
        IMG_H, IMG_W = label.shape
        interval = time.time() - last
        last = time.time()
        current_flag = 0
        roi = self.roi

        if total_time > 0 and total_time < 5:
            total_time += interval
            current_flag = pre_flag
        else:
            total_time = 0
            self.memory.append(flag+1)
            current_flag = self.get_flag()
            if current_flag != 0 and current_flag != pre_flag:
                pre_flag = current_flag
                total_time += interval
            else:
                pre_flag = current_flag

        y, x = self.get_point(label, current_flag, roi)

        while label[y][x] == 0 and roi < 0.9:
            roi += 0.05
            y, x = self.get_point(label, current_flag, roi)

        steer = np.arctan((x - IMG_W/2 + 1) / (IMG_H - float(y))) * 57.32
        steer = np.sign(steer) * min(60, abs(steer))
        #PID tuning
        #self.steer_pid.updateError(steer)
        #new_steer = -self.steer_pid.output()
        new_steer = -self.pid(x - IMG_W/2 + 1 )
        # print steer, new_steer
        # print self.steer_pid.Kp, self.steer_pid.Ki, self.steer_pid.Kd
        # twiddle_count_threshold = 500
        # if self.steer_pid.update_count > twiddle_count_threshold:
        #     self.steer_pid.twiddle()
        #if current_flag != 0 or pre_flag != 0:
            #speed = self.speed_min
        #else:
            #speed = max(self.speed_min, self.speed_max*np.cos(abs(steer)*np.pi/180))
        speed = (self.speed_min -self.speed_max) * steer**2/ 3600 + self.speed_max
            # speed = self.speed_max
        # label = self.write_vector(label, x, y, steer)
        return speed, steer, label
