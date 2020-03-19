# Run
# export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
# Before running this script to load graph faster

import tensorflow as tf
from tensorflow.python.compiler.tensorrt import trt_convert as trt

import time
from PIL import Image
import numpy as np

image = Image.open('test.png')
image.resize((224, 224))
input_data = np.array(image)


def read_pb_graph(pb_path):
    with tf.gfile.FastGFile(pb_path, 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    return graph_def

n_time_inference = 100


# input_img = np.zeros((1, 224, 224, 3))
input_img = np.expand_dims(input_data, 0)
input_img = input_img / 255.
print(input_img.shape)

tf.compat.v1.reset_default_graph()
with tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(gpu_options=tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.33))) as sess:
    print('Load graph..')
    # trt_graph = read_pb_graph('./model_dira.pb')
    trt_graph = read_pb_graph('./model_dira_TRTFP16.pb')

    print('Import graph..')
    tf.import_graph_def(
        trt_graph,
        name='',
        return_elements=['output1/Sigmoid:0'])

    print('Inference...')
    input = sess.graph.get_tensor_by_name('input_1:0')
    output = sess.graph.get_tensor_by_name('output1/Sigmoid:0')
    for k in range(4):
        start_clock = time.time()
        for i in range(n_time_inference):
            out_pred = sess.run(output, feed_dict={input: input_img})
        end_clock = time.time()

        inference_time = end_clock - start_clock
        fps = n_time_inference / inference_time
        print('FPS = {:.2f}'.format(fps))

    print(out_pred.squeeze(0).squeeze(2).shape)
    print(out_pred.max(), out_pred.min())

    out_pred = (out_pred.squeeze(0).squeeze(2) * 255).astype(np.uint8)
    print(out_pred.max(), out_pred.min())
    out_image = Image.fromarray(out_pred)
    out_image.show('Output')
