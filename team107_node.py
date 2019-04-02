#!/usr/bin/env python
# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import roslib

roslib.load_manifest('team107')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import model_from_json, load_model
import time
import rospkg
from steer import SegmentToSteer
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
end = time.time()
end_depth = time.time()
start = time.time()
check = True
is_running = True


# t1 = 0
class Processor:
    def __init__(self, model, sign_model):
        self.cv_bridge = CvBridge()
        self.image = None
        self.frame = 0
        self.flag = 0
        self.model = model
        self.sign_model = sign_model
        self.ss_sub = rospy.Subscriber('ss_status', Bool, self.run_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.callback,
                                          queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback, queue_size=1)
        self.pub_speed = rospy.Publisher('/set_speed_car_api', Float32, queue_size=1)
        self.pub_steerAngle = rospy.Publisher('/set_steer_car_api', Float32, queue_size=1)
        self.lastTime = time.time()
        self.s2s = SegmentToSteer(square=3, margin=20, roi=0.45)
        self.left_restriction = 0
        self.right_restriction = 319

    def depth_callback(self, data):
        global end_depth
        global is_running
        delta = time.time() - end_depth
        if delta >= 0.03 and is_running:
            try:
                # convert_data(data.data)
                img = self.cv_bridge.imgmsg_to_cv2(data, "passthrough")
                img = cv2.resize(img, (320, 160))
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
                # img *= 10
                # cv2.imshow('depth_left', img[90:159,  :int(0.25*IMG_W)])
                # cv2.imshow('depth', img)
            except CvBridgeError, e:
                print e
            end_depth = time.time()

    def check_obstacle_2(self, img, threshold):
        IMG_H, IMG_W = img.shape
        self.left_restriction = 0
        self.right_restriction = IMG_W - 1
        left_obs = False
        right_obs = False
        i = int(0.5 * IMG_W)
        range = int(0.05 * IMG_W)
        lower_y = 0
        upper_y = IMG_H
        size = (upper_y - lower_y) * range
        max_value = 255
        ratio = 0.5
        left_border = 0.2
        right_border = 1 - left_border

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
        elif self.left_restriction >= left_border * IMG_W and self.right_restriction <= right_border * IMG_W:
            self.left_restriction = 0
            self.right_restriction = IMG_W - 1

    def run_callback(self, data):
        global is_running
        is_running = data.data

    def callback(self, data):
        global end
        global is_running
        # global check
        # global start
        # global t1
        # if check == True:
        # 	start = time.time()
        # 	check = False
        delta = time.time() - end
        if delta >= 0.03 and is_running:
            try:
                self.image = self.convert_data_to_image(data.data)
                if self.frame % 8 == 0:
                    self.flag, s = self.sign_model.predict(self.image)
                    self.frame = 0
                self.image = cv2.resize(self.image, (320, 160))
                res, sharp = self.model.predict(self.image)
                # cv2.line(self.image, (self.left_restriction, 0), (self.left_restriction, 159), (0, 255, 0), 2)
                # cv2.line(self.image, (self.right_restriction, 0), (self.right_restriction, 159), (0, 255, 0), 2)
                # cv2.imshow('image', self.image)
                # cv2.waitKey(1)
                speed, steer, res = self.s2s.get_steer(self.image, res * 255., self.flag, sharp, self.left_restriction,
                                                       self.right_restriction)
                #cv2.imshow('black and white', res)
                #cv2.waitKey(1)
                # cv2.imshow('road', res)
                # cv2.waitKey(1)
                # if time.time() - start <= 10:
                # 	speed = 100
                # print (1/(time.time()-t1))

                self.publish_data(speed, -steer)
                # t1 = time.time()
                self.frame += 1
            except CvBridgeError as e:
                print(e)
            end = time.time()
        elif is_running == False:
            self.s2s.speed_memory = deque(iterable=np.zeros(5, dtype=np.uint8), maxlen=5)
            self.s2s.error_proportional_ = 0.0
            self.s2s.error_integral_ = 0.0
            self.s2s.error_derivative_ = 0.0
            self.publish_data(0, 0)

    def convert_data_to_image(self, data):
        arr = np.fromstring(data, np.uint8)
        image = cv2.imdecode(arr, 1)
        return image

    def get_segment_image(self, image):
        res = self.model.predict(np.expand_dims(image / 255., axis=0))
        return np.argmax(res, axis=3)[0]

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

