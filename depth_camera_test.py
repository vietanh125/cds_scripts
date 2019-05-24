#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
import rospkg
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import *
from tensorflow.keras import backend as K
from tensorflow.keras.models import model_from_json, load_model
from tensorflow.keras.utils import CustomObjectScope
from tensorflow.keras.initializers import glorot_uniform
from tensorflow.keras.models import Model
from tensorflow.keras import applications
from steer import SegmentToSteer
from depth_process import *
from floodfill import fill
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'

list_image = []
bridge = CvBridge()
rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
IMG_H = 160
IMG_W = 320
image = np.zeros((160, 320), np.float32)
color_image = np.zeros((160, 320), np.float32)
depth_image = None
seg_image = None
obstacle_time = 0.0
i_left, i_right = 0, 319
end = time.time()
skip = 100
# f = open(path + 'model_mobilenet_lane.json', 'r')
# with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
#     model = model_from_json(f.read())
#     model.load_weights(path + 'model-mobilenet-iter1234add-pretrain-bdd.h5')
#     model._make_predict_function()
#     print "Model loaded"

mbl = applications.mobilenet_v2.MobileNetV2(weights=None, include_top=False, input_shape=(160, 320, 3))
x = mbl.output
model_tmp = Model(inputs=mbl.input, outputs=x)
layer4, layer9, layer17 = model_tmp.get_layer('block_4_add').output, model_tmp.get_layer(
    'block_9_add').output, model_tmp.get_layer('out_relu').output
fcn18 = Conv2D(filters=2, kernel_size=1, name='fcn18')(layer17)
fcn19 = Conv2DTranspose(filters=layer9.get_shape().as_list()[-1], kernel_size=4, strides=2, padding='same',
                        name='fcn19')(fcn18)

fcn19_skip_connected = Add(name="fcn19_plus_vgg_layer9")([fcn19, layer9])
fcn20 = Conv2DTranspose(filters=layer4.get_shape().as_list()[-1], kernel_size=4, strides=2, padding='same',
                        name="fcn20_conv2d")(fcn19_skip_connected)

# Add skip connection
fcn20_skip_connected = Add(name="fcn20_plus_vgg_layer4")([fcn20, layer4])
# Upsample again
fcn21 = Conv2DTranspose(filters=2, kernel_size=16, strides=(8, 8), padding='same', name="fcn21", activation="softmax")(
    fcn20_skip_connected)
model1 = Model(inputs=mbl.input, outputs=fcn21)
model1.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
model1.load_weights(path + 'model-mobilenetv2-round1' + '.h5')
model1._make_predict_function()
print("Finish")
s2s = SegmentToSteer(roi=0.45, margin=10)
pre_turn_left = False
pre_turn_right = False
obstacle_time = 0.0
rects = []


def callback(data):
    global end, i_left, i_right, rects, ratio, skip
    if skip > 0:
        skip -= 1
        return
    try:
        # global image, color_image
        # convert_data(data.data)
        # i_left = 0
        # i_right = 319
        img = bridge.imgmsg_to_cv2(data)
        # print img.shape
        # img = cv2.resize(img, (320, 160))
        # i_l, i_r = check_obstacle_new(img)
        # print i_l, i_r
        # cv2.rectangle(color_image[lower_y:upper_y, border_x:-border_x],(x,y),(x+w,y+h),(0,255,0),2)
        # check_obstacle(thresh1, 0.5, delta)
        # print left_obstacle, right_obstacle
        # time2 = time.time()
        # print time2 - time1
        # # cv2.imshow('depth_left', img[90:159,  :int(0.25*IMG_W)])
        ratio = 8
        cut = 50
        # depth_preprocess(img, ratio)  # x, y, w, h
        # img = np.asarray(img, dtype=np.float32)
        # img *= 255./65535
        # img = np.asarray(img, dtype=np.uint16)
        # cv2.imshow("result", img * 10)
        # cv2.waitKey(1)
        t = time.time()
        depth_preprocess(img, ratio, cut)
        # print(rects)
        # img = cv2.resize(img, (640, 480))
        # cv2.imshow('raw', image*255.)
        # cv2.imshow('segment', color_image)
        # cv2.waitKey(1)
    except CvBridgeError, e:
        print
        e


def depth_preprocess(frame, ratio, cut):
    global i_left, i_right, rects, image, color_image
    h, w = frame.shape
    frame = preprocess_img(frame, ratio, cut)
    frame = remove_noise(frame, k=3)
    # print(frame)
    frame = remove_ground(frame, padding=1, distance=2)
    # print(frame)
    if image is not None:
        rects = find_obstacle(np.array(frame, np.uint8), image, k=3, min_area=0)
        for rect in rects:
            x, y, w, h = rect
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 1)
        print(rects)
        cv2.imshow('image', color_image)
        cv2.waitKey(1)
    # cv2.imshow("result", frame*255.)
    # cv2.waitKey(1)
    # rects = find_obstacles(frame, 3, ratio)
    # i_left, i_right = get_restriction(rects)


def depth_preprocess1(img):
    cv2.imshow('original', img * 10)
    global color_image
    lower_y = int(4 * 160 / 16)
    upper_y = int(12 * 160 / 16)
    img = img[lower_y:upper_y]
    img = np.float32(img)
    img *= 255.0 / 65535
    img = np.uint8(img)
    img = cv2.medianBlur(img, 5)
    # img *= 10
    # img[img == 0] = 255.0
    # img = remove_noise(img, 5)
    ret, thresh1 = cv2.threshold(img, 3, 255, cv2.THRESH_BINARY)
    cv2.imshow('color_image', color_image)
    cv2.imshow('threshold', thresh1)
    cv2.waitKey(1)
    check_obstacle_2(thresh1, 0.5)


def remove_noise(img, k):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    # img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return img


def obstackle_classify(pred, cnt):
    x, y, w, h = cnt
    # print cnt
    rec = pred[y: y + h, x: x + w]
    rec = cv2.Laplacian(rec, cv2.CV_64F)
    # print rec.shape
    rec = rec[1:-1, 1:-1]
    _, rec = cv2.threshold(rec, 0, 1, cv2.THRESH_BINARY)
    s = rec.sum()
    ratio = float(s / (h + w))
    if ratio > 0.6:
        return True
    return False


def get_restriction(rects):
    x_left = IMG_W // 2
    x_right = IMG_W // 2
    for rect in rects:
        x, y, w, h = rect
        # print(rect)
        x = x * 4
        w = w * 4
        # cv2.rectangle(image, (x, 0), (x+w, IMG_H), (0, 255, 0), 1)

        if x > IMG_W // 2:
            if x > x_right:
                x_right = x

        if (x + w) < IMG_W // 2:
            if (x + w) < x_left:
                x_left = x + w

        if x < IMG_W // 2 and x + w > IMG_W / 2:
            if IMG_W / 2 - x > x + w - IMG_W / 2:
                x_left = x + w
                x_right = IMG_W - 1
            else:
                x_left = 0
                x_right = x

    if x_right != IMG_W // 2:
        i_right = x_right
    else:
        i_right = IMG_W - 1

    if x_left != IMG_W // 2:
        i_left = x_left
    else:
        i_left = 0
    return i_left, i_right


def callback_image(data):
    # print 'lam on day viet anh'

    global skip, i_left, i_right, image, color_image, rects
    img = convert_data_to_image(data.data)
    if skip > 0:
        return
    cv2.line(img, (i_left, 0), (i_left, 159), (0, 255, 0), 2)
    cv2.line(img, (i_right, 0), (i_right, 159), (255, 0, 0), 2)

    # cv2.line(img, (0, int(6 * 160 / 16)), (0, int(6 * 160 / 16)), (255, 0, 0), 2)
    # cv2.line(img, (0, int(12 * 160 / 16)), (319, int(12 * 160 / 16)), (255, 0, 0), 2)
    color_image = img
    img_cpy = np.expand_dims(img / 255., axis=0)
    seg1 = model1.predict(img_cpy)[0]
    seg1 = np.argmax(seg1, axis=2)
    image = seg1
    color = (0, 0, 255)
    # if len(rects) > 0:
    #     for rect in rects:
    #         x, y, w, h = rect
    #
    #         if obstackle_classify(image * 1., rect):
    #             cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
    #         else:
    #             cv2.rectangle(img, (x, y), (x + w, y + h), color, 1)
    seg1 = fill(np.uint8(seg1))
    seg1 = seg1 * 255.
    image = seg1

    # _, _, res = s2s.get_steer(img, seg1, 0, 0, i_left, i_right)

    # cv2.imshow('image', img)
    # cv2.imshow('seg', res)
    cv2.waitKey(1)


def convert_data_to_image(data):
    arr = np.fromstring(data, np.uint8)
    image = cv2.imdecode(arr, 1)
    image = cv2.resize(image, (320, 160))
    return image


depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback, queue_size=1)
image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, callback_image, tcp_nodelay=True)
rospy.init_node('test')
rospy.spin()
