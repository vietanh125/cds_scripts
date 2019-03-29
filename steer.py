
import numpy as np
import time
import math
from collections import deque
total_time = 0
last = time.time()
pre_flag = 0
map = []
import cv2
total_distance = 0.0
last_speed = 0.0
first_tine = True
stored_time = 0.0
from new_steer_system import get_point_3


class SegmentToSteer():
    def __init__(self, square=7, margin=30, roi=1/3, size=1):
        self.square = square
        self.margin = margin
        self.size = size
        self.memory = deque(iterable=np.zeros(size, dtype=np.uint8), maxlen=size)
        self.roi = 1 - roi
        self.speed_max = 25
        self.speed_min = 15
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

    def get_point(self, img, flag, roi, left_restriction, right_restriction):
        IMG_H, IMG_W = img.shape
        i = int(IMG_H * roi)
        border = int((self.square - 1) / 2)
        i_l = left_restriction + border
        i_r = right_restriction - border
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
        if (turn_left and turn_right and flag == 1) or (turn_right and not turn_left):
            while img[i][i_r] == 255 and i >= 0:
                i -= 1
            return i+1, i_r
        elif (turn_left and turn_right and flag == -1) or (turn_left and not turn_right):
            while img[i][i_l] == 255 and i >= 0:
                i -= 1
            return i+1, i_l
        else:
            return i, int((i_l + i_r) / 2)

    def get_point_3(self, img, flag):
        IMG_H, IMG_W = img.shape
        count = 0
        turn_left = False
        turn_right = False
        x_left, y_left, x_right, y_right = 0, 0, 0, 0
        for i in range(int(IMG_H * 0.3), int(IMG_H * 0.8)):
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
        else:
			return int((y_left + y_right)/2), int((x_left + x_right)/2)

    def get_line(self, x1, y1, x2, y2):
        points = []
        issteep = abs(y2-y1) > abs(x2-x1)
        if issteep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        rev = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            rev = True
        deltax = x2 - x1
        deltay = abs(y2-y1)
        error = int(deltax / 2)
        y = y1
        ystep = None
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1
        for x in range(x1, x2 + 1):
            if issteep:
                points.append((y, x))
            else:
                points.append((x, y))
            error -= deltay
            if error < 0:
                y += ystep
                error += deltax
        # Reverse the list if the coordinates were reversed
        if rev:
            points.reverse()
        return points

    def check_obstacle(self, label, x, y, mul):
        line_left = self.get_line(84, 150, x, y)
        line_right = self.get_line(234, 150, x, y)
        border = int((self.square - 1) / 2)
        correction = 0
        skip = 0
        for point in line_left:
            if skip == border + 1:
                skip = 0
                continue
            check = label[point[1] - border: point[1] + border + 1, point[0] - border: point[0] + border + 1]
            if np.sum(check)/255 == 0:
                x_l = point[0]
                y_l = point[1]
                multiplier = mul
                if x < x_l:
                    multiplier *= -1
                while label[y_l, x_l] == 0 and x_l < 319:
                    x_l += 1
                correction += multiplier * int((x_l - 84) * y_l/159.0 * y/159.0)
                break
            skip += 1
        for point in line_right:
            if skip == border + 1:
                skip = 0
                continue
            check = label[point[1] - border: point[1] + border + 1, point[0] - border: point[0] + border + 1]
            if np.sum(check)/255 == 0:
                x_r = point[0]
                y_r = point[1]
                multiplier = mul
                if x > x_r:
                    multiplier *= -1
                while label[y_r, x_r] == 0 and x_r > 0:
                    x_r -= 1
                correction += multiplier * int((x_r - 234) * y_r/159.0 * y/159.0)
                break
            skip += 1
        return correction

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

    def mean_steer_queue(self):
        mean = 0
        for x in self.steer_queue:
            mean += x
        return mean/len(self.steer_queue)

    def get_steer(self, raw, label, flag, s, left_restriction, right_restriction):
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

        if total_time > 0 and total_time < 1:
            total_time += interval
            current_flag = pre_flag
        else:
            total_time = 0
            self.memory.append(flag+1)
            current_flag = self.get_flag()
            if current_flag != 0:
                pre_flag = current_flag
                total_time += interval
            else:
                pre_flag = current_flag

        # y, x = get_point_3(label, current_flag, 5)

        y, x = self.get_point(label, current_flag, roi, left_restriction, right_restriction)
		# y, x = self.get_point_2(label, current_flag)

        while label[y][x] == 0 and roi < 0.9:
            roi += 0.05
            y, x = self.get_point(label, current_flag, roi, left_restriction, right_restriction)

        # x = max(0, min(x + self.check_obstacle(label, x, y, 1), 319))
        # x = get_cte(label, current_flag, self.square, 1)
        steer = np.arctan((x - IMG_W/2 + 1) / (IMG_H - float(y))) * 57.32
        steer = np.sign(steer) * min(60, abs(steer))
        #PID tuning
        # self.steer_pid.updateError(x - IMG_W/2 + 1)
        # new_steer = -self.steer_pid.output()
        # new_steer = -self.pid(x - IMG_W/2 + 1 )
        # new_steer = -self.pid(steer/60) * 60
        #
        # new_steer = np.sign(new_steer) * min(60, abs(new_steer))

        # print steer, new_steer
        # print self.steer_pid.Kp, self.steer_pid.Ki, self.steer_pid.Kd
        # twiddle_count_threshold = 500
        # if self.steer_pid.update_count > twiddle_count_threshold:
        #     self.steer_pid.twiddle()
        #if current_flag != 0 or pre_flag != 0:
            #speed = self.speed_min
        #else:
            #speed = max(self.speed_min, self.speed_max*np.cos(abs(steer)*np.pi/180))
        # speed = (self.speed_min -self.speed_max) * new_steer**2/ 3600 + self.speed_max
            # speed = self.speed_max
        # label = self.write_vector(label, x, y, steer)
        # return speed, new_steer, label.update_count > twiddle_count_threshold:
        #     self.steer_pid.twiddle()
        #if current_flag != 0 or pre_flag != 0:
            #speed = self.speed_min
        #else:
            #speed = max(self.speed_min, self.speed_max*np.cos(abs(steer)*np.pi/180))
        speed = (self.speed_min -self.speed_max) * steer**2/ 3600 + self.speed_max
            # speed = self.speed_max
        label = self.write_vector(label, x, y, steer)
        return speed, steer, label
