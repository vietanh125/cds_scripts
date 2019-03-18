import tensorflow as tf
tf.keras.backend.set_learning_phase(0)
from tensorflow.python.platform import gfile
import tensorflow.contrib.tensorrt
import numpy as np
class Model():
    def __init__(self, path):
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)
        with self.graph.as_default():
            trt_graph = self.read_pb_graph(path)
            tf.import_graph_def(trt_graph, name='')
            self.input = self.sess.graph.get_tensor_by_name('input_1:0')
            self.output = self.sess.graph.get_tensor_by_name('fcn17/truediv:0')

    def read_pb_graph(self, path):
        with gfile.FastGFile(path, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        return graph_def
    def predict(self, image):
        image = np.expand_dims(image, 0)
        pred = self.sess.run(self.output, feed_dict={self.input: image/255.})
        pred = np.argmax(pred, axis=3)[0]
        return pred
