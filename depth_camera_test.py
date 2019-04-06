#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
import rospkg
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tensorflow.keras.models import model_from_json, load_model
from tensorflow.keras.utils import CustomObjectScope
from tensorflow.keras.initializers import glorot_uniform
from steer import SegmentToSteer

list_image = []
bridge = CvBridge()
rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
IMG_H = 160
IMG_W = 320
image = np.zeros((160, 320), np.float32)
color_image = np.zeros((160, 320), np.float32)
depth_image = None
i_left, i_right = 0, 319
end = time.time()
# f = open(path + 'model_mobilenet_lane.json', 'r')
# with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
#     model = model_from_json(f.read())
#     model.load_weights(path + 'model-mobilenet-iter3-pretrain-bdd.h5')
#     model._make_predict_function()
#     print "Model loaded"
# s2s = SegmentToSteer(roi=0.45)
pre_turn_left = False
pre_turn_right = False


def callback(data):
    global end
    delta = time.time() - end
    if delta >= 0.03:
        try:
            global image, color_image
            # convert_data(data.data)
            time1 = time.time()
            img = bridge.imgmsg_to_cv2(data, "passthrough")
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
            check_obstacle(thresh1, 0.5)
            time2 = time.time()
            print time2 - time1
            # # cv2.imshow('depth_left', img[90:159,  :int(0.25*IMG_W)])
            cv2.imshow('raw', color_image)
            cv2.imshow('segment', image)
            # # # cv2.imshow('sobel', sobel)
            cv2.imshow('threshold', thresh1)
            # cv2.imshow('depth', image)
            cv2.waitKey(1)
        except CvBridgeError, e:
            print e


def callback2(data):
    global end
    global list_image
    delta = time.time() - end
    if delta >= 0.03:
        try:
            global image, color_image
            # convert_data(data.data)
            # time1 = time.time()
            img = bridge.imgmsg_to_cv2(data, "passthrough")
            # img = cv2.GaussianBlur(img, (5, 5), 0)
            # img = img[:, 20:620]
            list_image.append(img)
            # img[img == 0] = 65535,
            cv2.imshow('depth', img)
            cv2.waitKey(1)
        except CvBridgeError, e:
            print e
        end = time.time()


def callback_image(data):
    global i_left, i_right, image, color_image
    img = convert_data_to_image(data.data)
    cv2.line(img, (i_left, 0), (i_left, 159), (0, 255, 0), 2)
    cv2.line(img, (i_right, 0), (i_right, 159), (255, 0, 0), 2)
    cv2.line(img, (0, int(6 * 160 / 16)), (0, int(6 * 160 / 16)), (255, 0, 0), 2)
    cv2.line(img, (0, int(12 * 160 / 16)), (319, int(12 * 160 / 16)), (255, 0, 0), 2)
    color_image = img
    # res = model.predict(np.expand_dims(img/255., axis=0))
    # res = np.argmax(res, axis=3)[0]
    # res = res * 255.
    # _, _, res = s2s.get_steer(img, res, 0, 0, i_left, i_right)
    # image = res
    # cv2.imshow('image', img)
    # cv2.waitKey(1)


def check_obstacle2(img, threshold):
    global i_left, i_right
    IMG_H = 160
    IMG_W = 320
    max_value = 255
    i_left = 0
    i_right = IMG_W - 1
    left_obs = False
    right_obs = False
    i = int(0.5 * IMG_W)
    range = int(0.1 * IMG_W)
    lower_y = int(9 * IMG_H / 16)
    size = (IMG_H - lower_y) * range
    while i - range >= 0.2 * IMG_W:
        if left_obs and right_obs:
            break
        if not left_obs:
            check = img[lower_y:IMG_H, i - range:i]
            left_check = np.sum(check) / (max_value * size)
            if left_check <= threshold:
                left_obs = True
                i_left = i
        if not right_obs:
            check = img[lower_y:IMG_H, IMG_W - i: IMG_W - i + range]
            right_check = np.sum(check) / (max_value * size)
            if right_check <= threshold:
                right_obs = True
                i_right = IMG_W - i
        i -= range


def check_obstacle(img, threshold):
    global i_left, i_right
    IMG_H, IMG_W = img.shape
    max_value = 255
    i_left = 0
    i_right = IMG_W - 1
    left_obs = False
    right_obs = False
    i = int(0.5 * IMG_W)
    range = int(0.1 * IMG_W)
    lower_y = 0
    upper_y = IMG_H
    size = (upper_y - lower_y) * range
    while i - range >= 0:
        if left_obs and right_obs:
            break
        if not left_obs:
            check = img[lower_y:upper_y, i - range:i]
            left_check = np.sum(check) / (max_value * size)
            if left_check <= threshold:
                left_obs = True
                i_left = i
        if not right_obs:
            check = img[lower_y:upper_y, IMG_W - i: IMG_W - i + range]
            right_check = np.sum(check) / (max_value * size)
            if right_check <= threshold:
                right_obs = True
                i_right = IMG_W - i
        i -= range
    if i_left >= i_right:
        i += range
        while np.sum(img[lower_y:upper_y, i_left - range:i_left]) / (
                max_value * size) <= threshold and i_left - range >= 0:
            i_left -= range
        while np.sum(img[lower_y:upper_y, i_right: i_right + range]) / (
                max_value * size) <= threshold and i_right + range <= IMG_W:
            i_right += range
        if i - i_left > i_right - i:
            i_left = i_right
            i_right = IMG_W - 1
        elif i - i_left < i_right - i:
            i_right = i_left
            i_left = 0
        else:
            i_left = 0
            i_right = IMG_W - 1
    # if i_left >= 0.2 * IMG_W and i_right >= 0.8 IMG_W:
    # if i_right <= 0.8 * IMG_W and i_lef:


def convert_data_to_image(data):
    arr = np.fromstring(data, np.uint8)
    image = cv2.imdecode(arr, 1)
    image = cv2.resize(image, (320, 160))
    return image


# def check_obstacle(image)

depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback, tcp_nodelay=True)
image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, callback_image, tcp_nodelay=True)
rospy.init_node('test')
rospy.spin()
