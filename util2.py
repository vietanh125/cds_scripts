#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
import os
import time
from rospkg import RosPack
from team107_node import Processor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32, Bool
from rospy import ROSException
from tensorflow.keras.layers import *
from tensorflow.keras.models import Model
from tensorflow.keras import applications

rospack = RosPack()

class Utilities:
    def __init__(self):
        #params
        self.count = 0
        self.clear_str = '0:0:                                                                             '
        self.is_recording = False
        self.is_self_driving = False
        self.is_pid_configuring = False
        self.is_changing_increment = False
        self.last_time = 0
        self.current_time = 0
        self.interval = 0
        self.engine = None
        self.path = rospack.get_path('team107') + '/scripts/'
        self.bt1_status = False
        self.bt2_status = False
        self.bt3_status = False
        self.bt4_status = False
        self.ss_status = False
        self.model = self.load_model_segment()
        #ros subscribers and publishers
        #hal
        self.sub_bt1 = rospy.Subscriber('/bt1_status', Bool, self.bt1_callback, queue_size=1)
        self.sub_bt2 = rospy.Subscriber('/bt2_status', Bool, self.bt2_callback, queue_size=1)
        self.sub_bt3 = rospy.Subscriber('/bt3_status', Bool, self.bt3_callback, queue_size=1)
        self.sub_bt4 = rospy.Subscriber('/bt4_status', Bool, self.bt4_callback, queue_size=1)
        self.sub_ss = rospy.Subscriber('/ss_status', Bool, self.ss_callback, queue_size=1)
        self.pub_lcd = rospy.Publisher('/lcd_print', String, queue_size='5')
        #car controller
        self.sub_img = None
        rospy.init_node('team107')
        print 'Finish initializing'
        #finish

    #setup callbacks
    def bt1_callback(self, data):
        self.bt1_status = data.data

    def bt2_callback(self, data):
        self.bt2_status = data.data

    def bt3_callback(self, data):
        self.bt3_status = data.data

    def bt4_callback(self, data):
        self.bt4_status = data.data

    def ss_callback(self, data):
        self.ss_status = data.data
        self.check_btns()

    def image_callback(self, ros_data):
        if not self.ss_status:
            tore_path = self.path + 'segment_data/'
            image = self.convert_to_image(ros_data.data)
            # image_path = self.path + str(self.count) + '.jpg'
            # cv2.imwrite(image_path, image)
            # cv2.waitKey(1)
            # self.count += 1

    #button interactions
    def check_btns(self):
		self.current_time = time.time()
		self.interval = self.current_time - self.last_time
        if not self.is_recording and not self.is_self_driving:
            if self.bt1_status and not self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.setup_record()
                self.record_control()
            elif not self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.setup_engine()
                self.engine_control()
        elif self.is_recording:
            self.record_control()
        elif self.is_self_driving:
            self.engine_control()

    #record functions
    def setup_record(self):
        self.is_recording = True
        self.is_self_driving = False
        self.bt1_status = False
        self.pub_lcd.publish(self.clear_str)
        self.sub_img = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.pub_lcd.publish('0:0:' + self.free_space('/home'))
        print 'start recording'
        time.sleep(1)

    def record_control(self):
        if self.bt1_status and self.bt2_status and self.bt3_status and not self.bt4_status:
            self.turn_off_record()

    def turn_off_record(self):
        self.is_recording = False
        self.sub_img.unregister()
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish("0:0:Finish recording data")
        print 'turnning off'
        time.sleep(1)

    #engine functions
    def setup_engine(self):
        self.is_self_driving = True
        self.is_recording = False
        self.bt2_status = False
        self.engine = Processor(model=self.model)
        self.print_car_stats()
        time.sleep(1)

    def engine_control(self):
        if self.is_pid_configuring:
            self.configuration_control()
        else:
            if self.bt1_status and self.bt2_status and self.bt3_status and not self.bt4_status:
                self.turn_off_engine()
            elif self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.setup_configuration()
                self.configuration_control()
            elif self.bt1_status and not self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.change_max_speed(1)
            elif not self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.change_min_speed(1)
            elif not self.bt1_status and not self.bt2_status and self.bt3_status and not self.bt4_status:
                self.change_roi(0.05)
            elif self.bt1_status and not self.bt2_status and not self.bt3_status and self.bt4_status:
                self.change_max_speed(-1)
            elif not self.bt1_status and self.bt2_status and not self.bt3_status and self.bt4_status:
                self.change_min_speed(-1)
            elif not self.bt1_status and not self.bt2_status and self.bt3_status and self.bt4_status:
                self.change_roi(-0.05)


    def change_max_speed(self, amount):
        self.engine.s2s.speed_max += amount
        self.print_car_stats()
        time.sleep(0.5)

    def change_min_speed(self, amount):
        self.engine.s2s.speed_min += amount
        self.print_car_stats()
        time.sleep(0.5)

    def change_roi(self, amount):
        change = self.engine.s2s.roi + amount
        self.engine.s2s.roi = max(0.1, min(change, 0.9))
        self.print_car_stats()
        time.sleep(0.5)

    def turn_off_engine(self):
        self.is_self_driving = False
        del self.engine
        self.engine = None
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish("0:0:Engine turned off")
        time.sleep(1)

    #pid configuration functions
    def setup_configuration(self):
        self.is_pid_configuring = True
        self.bt1_status = False
        self.bt2_status = False
        self.print_pid_stats()

    def configuration_control(self):
		
        if self.is_changing_increment:
            self.increment_control()
        else:
            if self.bt1_status and self.bt2_status and self.bt3_status and not self.bt4_status:
                self.exit_config()
            elif self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.setup_increment()
                self.increment_control()
            elif self.bt1_status and not self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.change_Kp(self.engine.s2s.inc_p)
            elif not self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
                self.change_Ki(self.engine.s2s.inc_i)
            elif not self.bt1_status and not self.bt2_status and self.bt3_status and not self.bt4_status:
                self.change_Kd(self.engine.s2s.inc_d)
            elif self.bt1_status and not self.bt2_status and not self.bt3_status and self.bt4_status:
                self.change_Kp(-self.engine.s2s.inc_p)
            elif not self.bt1_status and self.bt2_status and not self.bt3_status and self.bt4_status:
                self.change_Ki(-self.engine.s2s.inc_i)
            elif not self.bt1_status and not self.bt2_status and self.bt3_status and self.bt4_status:
                self.change_Kd(-self.engine.s2s.inc_d)


    def change_Kp(self, amount):
        self.engine.s2s.k_p += amount
        self.print_pid_stats()
        time.sleep(0.5)

    def change_Ki(self, amount):
        self.engine.s2s.k_i += amount
        self.print_pid_stats()
        time.sleep(0.5)

    def change_Kd(self, amount):
        self.engine.s2s.k_i += amount
        self.print_pid_stats()
        time.sleep(0.5)

    def exit_config(self):
        self.is_pid_configuring = False
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish("0:0:Exit Configuration")
        time.sleep(1)
        self.print_car_stats()

    #increment configuration
    def setup_increment(self):
        self.is_pid_configuring = True
        self.bt1_status = False
        self.bt2_status = False
        self.print_increment()

    def increment_control(self):
        if not self.bt1_status and not self.bt2_status and not self.bt3_status and self.bt4_status:
            self.exit_increment()
        elif self.bt1_status and not self.bt2_status and not self.bt3_status and not self.bt4_status:
            self.change_inc_p(0)
        elif not self.bt1_status and self.bt2_status and not self.bt3_status and not self.bt4_status:
            self.change_inc_i(0)
        elif not self.bt1_status and not self.bt2_status and self.bt3_status and not self.bt4_status:
            self.change_inc_d(0)
        elif self.bt1_status and not self.bt2_status and not self.bt3_status and self.bt4_status:
            self.change_inc_p(1)
        elif not self.bt1_status and self.bt2_status and not self.bt3_status and self.bt4_status:
            self.change_inc_i(1)
        elif not self.bt1_status and not self.bt2_status and self.bt3_status and self.bt4_status:
            self.change_inc_d(1)

    def change_inc_p(self, operator):
        if operator == 0:
            self.engine.s2s.inc_p *= 10
        else:
            self.engine.s2s.inc_p /= 10
        self.pub_lcd.publish(self.clear_str)
        self.print_increment()
        time.sleep(0.5)

    def change_inc_i(self, operator):
        if operator == 0:
            self.engine.s2s.inc_i *= 10
        else:
            self.engine.s2s.inc_i /= 10
        self.pub_lcd.publish(self.clear_str)
        self.print_increment()
        time.sleep(0.5)

    def change_inc_d(self, operator):
        if operator == 0:
            self.engine.s2s.inc_d *= 10
        else:
            self.engine.s2s.inc_d /= 10
        self.pub_lcd.publish(self.clear_str)
        self.print_increment()
        time.sleep(0.5)

    def exit_increment(self):
        self.is_changing_increment = False
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish("0:0:Exit increment config")
        time.sleep(1)
        self.print_pid_stats()

    #utility functions
    def load_model_segment(self):
		mbl = applications.mobilenet.MobileNet(weights=None, include_top=False, input_shape=(160,320,3))
		x = mbl.output
		model_tmp =  Model(inputs = mbl.input, outputs = x)
		layer5, layer8, layer13 = model_tmp.get_layer('conv_pw_5_relu').output, model_tmp.get_layer('conv_pw_8_relu').output, model_tmp.get_layer('conv_pw_13_relu').output

		fcn14 = Conv2D(filters=2 , kernel_size=1, name='fcn14')(layer13)
		fcn15 = Conv2DTranspose(filters=layer8.get_shape().as_list()[-1] , kernel_size=4, strides=2, padding='same', name='fcn15')(fcn14)
		fcn15_skip_connected = Add(name="fcn15_plus_vgg_layer8")([fcn15, layer8])
		fcn16 = Conv2DTranspose(filters=layer5.get_shape().as_list()[-1], kernel_size=4, strides=2, padding='same', name="fcn16_conv2d")(fcn15_skip_connected)
		# Add skip connection
		fcn16_skip_connected = Add(name="fcn16_plus_vgg_layer5")([fcn16, layer5])
		# Upsample again
		fcn17 = Conv2DTranspose(filters=2, kernel_size=16, strides=(8, 8), padding='same', name="fcn17", activation="softmax")(fcn16_skip_connected)
		m = Model(inputs = mbl.input, outputs = fcn17)
		m.load_weights(self.path + 'model-mobilenet-iter2-pretrain-data-bdd.h5')
		m.predict(np.zeros((1, 160, 320, 3), dtype=np.float32))
		print("Model loaded")
		return m

    def convert_to_image(self, data):
        arr = np.fromstring(data, np.uint8)
        image = cv2.imdecode(arr, 1)
        image = cv2.resize(image, (320, 160))
        return image

    def free_space(self, path):
        st = os.statvfs(path)
        free = st.f_bavail * st.f_frsize
        return "free " + str(free/(1000*1000)) + "MB"

    def print_car_stats(self):
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish('0:0:max ' + str(self.engine.s2s.speed_max))
        self.pub_lcd.publish('7:0:min ' + str(self.engine.s2s.speed_min))
        self.pub_lcd.publish('0:2:roi ' + str(self.engine.s2s.roi))

    def print_pid_stats(self):
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish('0:O:P ' + str(self.engine.s2s.k_p))
        self.pub_lcd.publish('9:0:I ' + str(self.engine.s2s.k_i))
        self.pub_lcd.publish('0:2:D ' + str(self.engine.s2s.k_d))

    def print_increment(self):
        self.pub_lcd.publish(self.clear_str)
        self.pub_lcd.publish('0:O:iP ' + str(self.engine.s2s.inc_p))
        self.pub_lcd.publish('9:0:iI ' + str(self.engine.s2s.inc_i))
        self.pub_lcd.publish('0:2:iD ' + str(self.engine.s2s.inc_d))

def main(args):
    util = Utilities()
    try:
        rospy.spin()
    except:
        print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
