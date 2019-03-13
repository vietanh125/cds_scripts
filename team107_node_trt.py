#!/usr/bin/env python
# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import roslib
roslib.load_manifest('team107')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
tf.keras.backend.set_learning_phase(0)

from tensorflow.keras.models import model_from_json, load_model
from signRecognition import detect1
import time
import rospkg
from steer import SegmentToSteer


rospack = rospkg.RosPack()
path = rospack.get_path('team107') + '/scripts/'
end = time.time()
check = True

import tensorflow.contrib.tensorrt as trt
from tensorflow.python.platform import gfile
PRECISION = "FP32"

def read_pb_graph(model):
    with gfile.FastGFile(path+model,'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    return graph_def

graph = tf.Graph()
graph.as_default()
sess = tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0.5)))
trt_graph = read_pb_graph('./tensorRT/TensorRT_'+ PRECISION + '.pb')
tf.import_graph_def(trt_graph, name='')
input = sess.graph.get_tensor_by_name('input_1:0')
output = sess.graph.get_tensor_by_name('fcn17/truediv:0')
class processor:
	def __init__(self):
		self.image = None
		self.model = self.load_model_segment()
		self.sess = sess
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/Team1_image/compressed', CompressedImage, self.callback, queue_size=1)
		self.pub_speed = rospy.Publisher('/Team1_speed', Float32, queue_size=1)
		self.pub_steerAngle = rospy.Publisher('/Team1_steerAngle', Float32, queue_size=1)
		self.lastTime = time.time()
		self.s2s = SegmentToSteer(square=7, margin=10, roi=0.5)


	def callback(self, data):
		global end
		if time.time() - end >= 0.5:
			try:
				t1 = time.time()
				self.image = self.convert_data_to_image(data.data)
				flag, s = -1, 40
				image = self.image/255.
				image = np.expand_dims(image, 0)
				res = sess.run(output, feed_dict={input: image})
				res = np.argmax(res, axis=3)[0]
				cv2.imshow('black and white', res*255.)
				cv2.waitKey(1)
				speed, steer, res = self.s2s.get_steer(self.image, res*255., flag, s)
				cv2.imshow('road', res)
				cv2.waitKey(1)
				self.publish_data(speed, steer)
				print (1/(time.time() - t1))
			except CvBridgeError as e:
				print(e)
		end = time.time()

	def convert_data_to_image(self, data):
		arr = np.fromstring(data, np.uint8)
		image = cv2.imdecode(arr, 1)
		image = cv2.resize(image, (320,160))
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
