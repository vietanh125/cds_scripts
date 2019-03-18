#!/usr/bin/env python
# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import roslib
roslib.load_manifest('team107')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import model_from_json, load_model
#from signRecognition import detect1
from detection import detect
import time
import rospkg
from steer import SegmentToSteer

rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
end = time.time()
start = time.time()
check = True
is_running = True
t1 = 0
class Processor:
	def __init__(self, model):
		self.image = None
		self.model = model
		self.ss_sub = rospy.Subscriber('ss_status', Bool, self.run_callback, queue_size=1)
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.callback, queue_size=1)
		self.pub_speed = rospy.Publisher('/set_speed_car_api', Float32, queue_size=1)
		self.pub_steerAngle = rospy.Publisher('/set_steer_car_api', Float32, queue_size=1)
		self.lastTime = time.time()
		self.s2s = SegmentToSteer(square=3, margin=10, roi=0.3)

	def run_callback(self, data):
		global is_running
		is_running = data.data

	def callback(self, data):
		global end
		global is_running
		global check
		global start
		global t1
		# if check == True:
		# 	start = time.time()
		# 	check = False
		delta = time.time() - end
		if delta >= 0.03 and is_running == True:
			try:
				self.image = self.convert_data_to_image(data.data)
				flag, s = detect_3_channels(self.image)
				res = self.model.predict(self.image)
				cv2.imshow('image', self.image)
				#cv2.imshow('black and white', res*255.)
				cv2.waitKey(1)
				speed, steer, res = self.s2s.get_steer(self.image, res*255., flag, s)
				#cv2.imshow('road', res)
				#cv2.waitKey(1)
				# if time.time() - start <= 10:
				# 	speed = 100
				print (1/(time.time()-t1))
				self.publish_data(speed, -steer)
				t1 = time.time()
			except CvBridgeError as e:
				print(e)
			end = time.time()
		elif is_running == False:
			self.s2s.error_proportional_ = 0.0
			self.s2s.error_integral_     = 0.0
			self.s2s.error_derivative_   = 0.0
			self.publish_data(0, 0)

	def convert_data_to_image(self, data):
		arr = np.fromstring(data, np.uint8)
		image = cv2.imdecode(arr, 1)
		image = cv2.resize(image, (320,160))
		return image

	def get_segment_image(self, image):
		res = self.model.predict(np.expand_dims(image/255., axis=0))
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
