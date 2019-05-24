#!/usr/bin/env python
# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import roslib

roslib.load_manifest('team107')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import CompressedImage, Image, Imu
import numpy as np

import time
import rospkg
from steer import SegmentToSteer
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from depth_process import *
from floodfill import fill
from find_path import get_command

rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
end_depth = time.time()
start = time.time()
check = True
is_running = True
# t1 = 0
skip = 200
depth_skip = 200


class Processor:
    def __init__(self, model):
        self.cv_bridge = CvBridge()
        self.image = None
        self.pred_image = None
        self.frame = 0
        self.flag = 0
        self.model = model
        # self.sign_model = sign_model
        self.ss_sub = rospy.Subscriber('ss_status', Bool, self.run_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.callback,
                                          queue_size=1)
        self.imu_sub = rospy.Subscriber('/mpu_9250/imu', Imu, self.imu_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback, queue_size=1)
        self.pub_speed = rospy.Publisher('/set_speed_car_api', Float32, queue_size=1)
        self.pub_steerAngle = rospy.Publisher('/set_steer_car_api', Float32, queue_size=1)
        self.lastTime = time.time()
        self.s2s = SegmentToSteer(square=3, margin=20, roi=0.4)
        self.left_restriction = 0
        self.right_restriction = 319
        self.obstacle_time = 0.0
        # self.total_time = 0.0
        self.total_time_thresh = 8.0	

    def depth_callback(self, data):
        global is_running
        global depth_skip
        # if delta >= 0.03 and slf.total_time < self.total_time_thresh:
        if depth_skip > 0:
            depth_skip -= 1
            return
        try:
            # convert_data(data.data)
            if self.s2s.depth_time:
                img = self.cv_bridge.imgmsg_to_cv2(data, "passthrough")
                ratio = 8
                cut = 50
                self.depth_preprocess(img, ratio, cut)
                depth_skip += 1
        except CvBridgeError, e:
            print e

    def depth_preprocess(self, frame, ratio, cut):
        h, w = frame.shape
        frame = preprocess_img(frame, ratio, cut)
        frame = remove_noise(frame, k=3)
        # print(frame)
        frame = remove_ground(frame, padding=1, distance=2)
        # print(frame)
        if self.pred_image is not None:
            rects = find_obstacle(np.array(frame, np.uint8), self.pred_image, k=3, min_area=0)
            self.left_restriction, self.right_restriction = get_restriction_2(rects)
        # frame = np.float32(frame) * 255.
        # if r is not None:
        #     x, y, w, h = r
        #     print r
        #     cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 1)
        #
        # cv2.imshow('thresh', self.image)
        # cv2.waitKey(1)

    def depth_preprocess_hieu(self, img):
        lower_y = int(4 * 160 / 16)
        upper_y = int(12 * 160 / 16)
        img = img[lower_y:upper_y]
        img = np.float32(img)
        img *= 255.0 / 65535
        img = np.uint8(img)
        img = cv2.medianBlur(img, 5)
        # img *= 10
        ret, thresh1 = cv2.threshold(img, 3, 255, cv2.THRESH_BINARY)
        self.check_obstacle_2(thresh1, 0.5)

    def check_obstacle_2(self, img, threshold):
        IMG_H, IMG_W = img.shape
        self.left_restriction = 0
        self.right_restriction = IMG_W - 1
        left_obs = False
        right_obs = False
        i = int(0.5 * IMG_W)
        range = int(0.1 * IMG_W)
        lower_y = 0
        upper_y = IMG_H
        size = (upper_y - lower_y) * range
        max_value = 255
        ratio = 0.6
        left_border = 0.15
        right_border = 1 - left_border

        self.obstacle_time = 0.0
        while i - range >= 0:
            if left_obs and right_obs:
                break
            if not left_obs:
                check = img[lower_y:upper_y, i - range:i]
                left_check = np.sum(check) / (max_value * size)
                if left_check <= threshold:
                    left_obs = True
                    self.left_restriction = i
            if not right_obs:
                check = img[lower_y:upper_y, IMG_W - i: IMG_W - i + range]
                right_check = np.sum(check) / (max_value * size)
                if right_check <= threshold:
                    right_obs = True
                    self.right_restriction = IMG_W - i
            i -= range
        if self.left_restriction >= self.right_restriction:
            i += range
            while np.sum(img[lower_y:upper_y, self.left_restriction - range:self.left_restriction]) / (
                    max_value * size) <= threshold and self.left_restriction - range >= 0:
                self.left_restriction -= range
            while np.sum(img[lower_y:upper_y, self.right_restriction: self.right_restriction + range]) / (
                    max_value * size) <= threshold and self.right_restriction + range <= IMG_W:
                self.right_restriction += range
            if i - self.left_restriction > self.right_restriction - i:
                self.left_restriction = int(ratio * IMG_W)
                self.right_restriction = IMG_W - 1
            elif i - self.left_restriction < self.right_restriction - i:
                self.right_restriction = int((1 - ratio) * IMG_W)
                self.left_restriction = 0
            else:
                self.left_restriction = 0
                self.right_restriction = IMG_W - 1
        elif self.left_restriction >= left_border * IMG_W and self.right_restriction >= right_border * IMG_W:
            self.left_restriction = int(ratio * IMG_W)
            self.right_restriction = IMG_W - 1
        elif self.right_restriction <= right_border * IMG_W and self.left_restriction <= left_border * IMG_W:
            self.right_restriction = int((1 - ratio) * IMG_W)
            self.left_restriction = 0

    def parking(self, frame):
        H, W = 160, 320
        frame = frame[int(0.4 * H):-1, int(0.35 * W): int(0.65 * W)]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        _, frame = cv2.threshold(frame, 240, 255, cv2.THRESH_BINARY)
        # sobely = cv2.Sobel(frame, cv2.CV_8U, 0, 1, ksize=3)
        sobely = cv2.erode(frame, (3, 40))
        # cv2.imshow('line', sobely)
        # cv2.waitKey(1)
        i = 0
        h, w = sobely.shape
        while (i < h):
            if sobely[i][w / 2] == 255:
                break
            i += 1
        d = max(0, (h - i) - 50)
        if i >= h - 1:
            return -1
        return d / 3

    def imu_callback(self, imu):
        global skip
        if skip > 0:
            return
        self.s2s.append_tilt(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z)

    def run_callback(self, data):
        global is_running
        is_running = data.data

    def callback(self, data):
        global skip
        if skip > 0:
            skip -= 1
            return
        global is_running
        # global check
        # global start
        # global t1
        # if check == True:
        #   start = time.time()
        #   check = False
        try:
            self.image = self.convert_data_to_image(data.data)
            # if self.frame % 6 == 0:
            #     self.flag, s = self.sign_model.predict(self.image)
            #     self.frame = 0
            #     print self.flag
            self.image = cv2.resize(self.image, (320, 160))
            res, sharp = self.model.predict(self.image)
            res = fill(np.uint8(res))
            self.pred_image = res * 1.
            # print self.left_restriction, self.right_restriction
            speed, steer = self.s2s.get_steer(res, self.flag, sharp, self.left_restriction, self.right_restriction)

            if self.s2s.park_time and not self.s2s.on_crossroad:
                parking_speed = self.parking(self.image)
                if parking_speed > -1:
                    self.s2s.speed_memory.pop()
                    self.s2s.speed_memory.append(parking_speed)
                    speed = self.s2s.mean_speed_queue()
                    speed -= 1
            # cv2.imshow('black and white', self.image)
            # cv2.waitKey(1)
            cv2.imshow('road', res * 255.)
            cv2.waitKey(1)
            # print (1 / (time.time() - t1))
            if is_running:
                self.publish_data(speed, -steer)
            else:
                # self.s2s.total_width = self.s2s.roi * 160
                self.s2s.total_time_steer = 0.0
                self.total_time = 0.0
                self.s2s.mode = 0
                self.s2s.counter = 0
                self.s2s.reset_actions()
                self.s2s.speed_memory = deque(iterable=np.zeros(5, dtype=np.uint8), maxlen=5)
                self.s2s.error_proportional_ = 0.0
                self.s2s.error_integral_ = 0.0
                self.s2s.error_derivative_ = 0.0
                self.publish_data(0, 0)
            # print 1/(time.time() - t1)
            # t1 = time.time()
            # self.frame += 1
        except CvBridgeError as e:
            print(e)

    def convert_data_to_image(self, data):
        arr = np.fromstring(data, np.uint8)
        image = cv2.imdecode(arr, 1)
        return image

    def publish_data(self, speed, steerAngle):
        self.pub_speed.publish(speed)
        self.pub_steerAngle.publish(steerAngle)


def main(args):
    p = processor()
    rospy.init_node('team107')
    try:
        rospy.spin()
    except:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

