import tensorflow as tf
tf.keras.backend.set_learning_phase(0)
MODEL_PATH = "./tensorRT/model"
import tensorflow.contrib.tensorrt as trt
from tensorflow.python.platform import gfile
import cv2
import numpy as np
import time
from tensorflow.keras.models import model_from_json
from tensorflow.keras.utils import CustomObjectScope
from tensorflow.keras.initializers import glorot_uniform
from tensorflow.keras.layers import *
from tensorflow.keras.models import Model
from tensorflow.keras import applications
PRECISION = "FP16"
def load():
    mbl = applications.mobilenet.MobileNet(weights=None, include_top=False, input_shape=(160, 320, 3))
    x = mbl.output
    model_tmp = Model(inputs=mbl.input, outputs=x)
    layer5, layer8, layer13 = model_tmp.get_layer('conv_pw_5_relu').output, model_tmp.get_layer(
        'conv_pw_8_relu').output, model_tmp.get_layer('conv_pw_13_relu').output
    fcn14 = Conv2D(filters=2, kernel_size=1, name='fcn14')(layer8)
    fcn16 = Conv2DTranspose(filters=layer5.get_shape().as_list()[-1], kernel_size=4, strides=2, padding='same',
                            name="fcn16_conv2d")(fcn14)
    # Add skip connection
    fcn16_skip_connected = Add(name="fcn16_plus_vgg_layer5")([fcn16, layer5])
    # Upsample again
    fcn17 = Conv2DTranspose(filters=2, kernel_size=16, strides=(8, 8), padding='same', name="fcn17",
                            activation="softmax")(fcn16_skip_connected)
    model = Model(inputs=mbl.input, outputs=fcn17)
    model.load_weights('model-mobilenet-1M-iter12-pretrain-bdd.h5')


def load_keras_model(file_name):
    f = open(file_name + '.json', 'r')
    with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
        model = model_from_json(f.read())
    model.load_weights(file_name + '.h5')
    print "Model loaded"
    return model

def keras_to_TF():
    saver = tf.train.Saver()
    sess = tf.keras.backend.get_session()
    save_path = saver.save(sess, "./tensorRT/model")
    print("Keras model is successfully converted to TF graph in " + save_path)


def TF_to_TRT():
    with tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0))) as sess:
        saver = tf.train.import_meta_graph("./tensorRT/model.meta")
        saver.restore(sess, "./tensorRT/model")
        your_outputs = ["fcn17/truediv"]

        frozen_graph = tf.graph_util.convert_variables_to_constants(sess, tf.get_default_graph().as_graph_def(),
                                                                    output_node_names=your_outputs)
        with gfile.FastGFile("./tensorRT/frozen_model.pb", 'wb') as f:
            f.write(frozen_graph.SerializeToString())
        print("Frozen model is successfully stored!")
    trt_graph = trt.create_inference_graph(
        input_graph_def=frozen_graph,
        outputs=your_outputs,
        max_batch_size=1,
        max_workspace_size_bytes=1>>25,
        precision_mode=PRECISION)

    with gfile.FastGFile("./tensorRT/TensorRT_1M_" + PRECISION + ".pb", 'wb') as f:
        f.write(trt_graph.SerializeToString())
    print("TensorRT model is successfully stored!")
    all_nodes = len([1 for n in frozen_graph.node])
    print("numb. of all_nodes in frozen graph:", all_nodes)
    trt_engine_nodes = len([1 for n in trt_graph.node if str(n.op) == 'TRTEngineOp'])
    print("numb. of trt_engine_nodes in TensorRT graph:", trt_engine_nodes)
    all_nodes = len([1 for n in trt_graph.node])
    print("numb. of all_nodes in TensorRT graph:", all_nodes)
    return trt_graph

def read_pb_graph(model):
    with gfile.FastGFile(model,'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    return graph_def

def test(n_time_inference=50):
    input_img = np.zeros((1, 160, 320, 3))
    graph = tf.Graph()
    with graph.as_default():
        with tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0.5))) as sess:
            trt_graph = read_pb_graph('./tensorRT/TensorRT_1M_'+ PRECISION + '.pb')
            tf.import_graph_def(trt_graph, name='')
            input = sess.graph.get_tensor_by_name('input_1:0')
            output = sess.graph.get_tensor_by_name('fcn17/truediv:0')
            total_time = 0;
            out_pred = sess.run(output, feed_dict={input: input_img})
            for i in range(n_time_inference):
                t1 = time.time()
                out_pred = sess.run(output, feed_dict={input: input_img})
                t2 = time.time()
                delta_time = t2 - t1
                total_time += delta_time
            avg_time_tensorRT = total_time / n_time_inference
            print "average inference time: ", avg_time_tensorRT

    graph = tf.Graph()
    with graph.as_default():
        with tf.Session() as sess:
            frozen_graph = read_pb_graph('./tensorRT/frozen_model.pb')
            tf.import_graph_def(frozen_graph, name='')
            input = sess.graph.get_tensor_by_name('input_1:0')
            output = sess.graph.get_tensor_by_name('fcn17/truediv:0')
            total_time = 0;
            out_pred = sess.run(output, feed_dict={input: input_img})
            for i in range(n_time_inference):
                t1 = time.time()
                out_pred = sess.run(output, feed_dict={input: input_img})
                t2 = time.time()
                delta_time = t2 - t1
                total_time += delta_time
            avg_time_original_model = total_time / n_time_inference
            print "average inference time: ", avg_time_original_model
            print "TensorRT improvement compared to the original model:", avg_time_original_model / avg_time_tensorRT

def inference(sess, frame, input, output):
    gray = np.expand_dims(frame, axis=0)
    out_pred = sess.run(output, feed_dict={input: gray/255.})
    out_pred = np.argmax(out_pred, axis=3)[0]
    return out_pred

def inference_2(trt_graph):
    with tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0.0))) as sess:
        tf.import_graph_def(trt_graph, name='')
        input = sess.graph.get_tensor_by_name('input_1:0')
        output = sess.graph.get_tensor_by_name('fcn17/truediv:0')
        cap = cv2.VideoCapture('output.avi')
        total_time = 0
        ret, frame = cap.read()
        frame = cv2.resize(frame[:500, :], (320, 160))
        gray = np.expand_dims(frame, axis=0)
        out_pred = sess.run(output, feed_dict={input: gray/255.})
        n_frame = 1
        while (True):
            t1 = time.time()
            frame = cv2.resize(frame[:500, :], (320, 160))
            gray = np.expand_dims(frame, axis=0)
            out_pred = sess.run(output, feed_dict={input: gray/255.})
            out_pred = np.argmax(out_pred, axis=3)[0]
            # out_pred = inference(sess, frame, input, output)
            t2 = time.time()

            frame[out_pred == 1] = [0, 0, 255]
            delta_time = t2 - t1
            total_time += delta_time
            if n_frame % 50 == 0:
                print n_frame/total_time
            cv2.imshow('frame', frame)
            ret, frame = cap.read()
            if (cv2.waitKey(1) & 0xFF == ord('q')) or frame is None:
                break
            n_frame += 1



def pipe_line(keras_model_path):
    # load_keras_model(keras_model_path)
    # load()
    # keras_to_TF()
    # TF_to_TRT()
    test()

#pipe_line("model-mobilenet-iter2-pretrain-data-bdd")


sess = tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0.5)))
trt = read_pb_graph("TensorRT_1M_FP16.pb")
inference_2(trt)

