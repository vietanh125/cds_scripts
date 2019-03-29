#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

IMG_H = 160
IMG_W = 320
image = None
depth_image = None
i_left, i_right = 0, 319

def callback(data):
    try:
        # convert_data(data.data)
        time1 = time.time()
        img = bridge.imgmsg_to_cv2(data, "passthrough")
        #img = cv2.resize(img, (320, 160))
        check_obstacle(img)
        time2 = time.time()
        print time2 - time1
        # img *= 10
        # cv2.imshow('depth_left', img[90:159,  :int(0.25*IMG_W)])
        # cv2.imshow('depth', img)
        # cv2.waitKey(1)
    except CvBridgeError, e:
        print e

def callback_image(data):
    global i_left, i_right
    img = convert_data_to_image(data.data)
    cv2.line(img, (i_left, 0), (i_left, 159), (0, 255, 0), 2)
    cv2.line(img, (i_right, 0), (i_right, 159), (255, 0, 0), 2)
    # cv2.rectangle(img, (int(0.1*IMG_W), 90), (int(0.25*IMG_W), 159), (0, 255, 0), 2)
    # cv2.rectangle(img, (int(0.75*IMG_W), 90), (int(0.9*IMG_W), 159), (0, 255, 0), 2)
    cv2.imshow('image', img)
    cv2.waitKey(1)

def check_obstacle(img):
    global i_left, i_right
    left_obs = False
    right_obs = False
    i = 96
    i_left = 0
    i_right = 319
    while i - 16 >= 0:
        if left_obs and right_obs:
            break
        if not left_obs:
            check = img[90:160, i-16:i]
            left_check = np.sum(check)/(65535 * 1120)
            if left_check <= 0.01:
                left_obs = True
                i_left = i
        if not right_obs:
            check = img[90:160,  320 - i: 320 - i + 16]
            right_check = np.sum(check)/(65535 * 1120)
            if right_check <= 0.01:
                right_obs = True
                i_right = 320 - i
        i -= 16


def convert_data_to_image(data):
    arr = np.fromstring(data, np.uint8)
    image = cv2.imdecode(arr, 1)
    image = cv2.resize(image, (320,160))
    return image

# def check_obstacle(image)
        
depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback, tcp_nodelay=True)
#image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, callback_image, tcp_nodelay=True)
rospy.init_node('test')
rospy.spin()
