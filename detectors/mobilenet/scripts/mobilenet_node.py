#!/usr/bin/env python3
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def data_to_image(data):
    np_arr = np.fromstring(data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np


def image_to_data(image_np):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    return msg

class mobilenet_node():
    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic')
        self.weight_path = rospy.get_param('~weight_path')
        self.is_use_deep = rospy.get_param('~use_deep')

        if self.is_use_deep:
            self.init_model()
            self.proc_func = self.use_deep
        else:
            self.proc_func = self.use_imgproc

        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            self.image_topic, CompressedImage, self.img_callback, queue_size=1, buff_size=2**24)
        self.pub_lane = rospy.Publisher(
            '~lane_seg/compressed', CompressedImage, queue_size=1)

        # print('Inference')
        # self.image = cv2.imread('/home/ubuntu/catkin_ws/src/dira/camera_record/test.png')
        # output = self.use_deep(self.image)
        # n = 100
        # for k in range(4):
        #     now = time.time()
        #     for i in range(n):
        #         output = self.use_deep(self.image)
        #     fps = float(n)/(time.time()-now)
        #     print("FPS = ",fps)

        # cv2.imshow('output', output)
        # cv2.waitKey(0)

        rospy.set_param('~ready', True)
        rospy.loginfo('mobilenet init done')

    def init_model(self):

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.log_device_placement = False
        config.gpu_options.per_process_gpu_memory_fraction = 0.4
        
        with tf.gfile.FastGFile(self.weight_path, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        self.sess = tf.compat.v1.Session(config=config)
        tf.import_graph_def(
            graph_def,
            name='',
            return_elements=['output1/Sigmoid:0']
        )

    def use_deep(self, cv_image):
        img = cv2.resize(cv_image, (224, 224))
        img = img.squeeze() / 255.
        img = np.expand_dims(img, 0)

        input = self.sess.graph.get_tensor_by_name('input_1:0')
        output = self.sess.graph.get_tensor_by_name('output1/Sigmoid:0')
        out_pred = self.sess.run(output, feed_dict={input: img})

        out_pred = (out_pred.squeeze(0).squeeze(2) * 255).astype(np.uint8)
        return out_pred

    def use_imgproc(self, cv_image):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, cv_image = cv2.threshold(cv_image, 180, 255, cv2.THRESH_BINARY)
        return cv_image

    def img_callback(self, data):
        try:
            cv_image = data_to_image(data.data)
        except:
            print('Cannot convert to cv_image')
            return

        cv_image = self.proc_func(cv_image)
        lane_seg = image_to_data(cv_image)
        self.pub_lane.publish(lane_seg)


if __name__ == "__main__":
    rospy.init_node('mobilenet_node')
    node = mobilenet_node()
    rospy.spin()
    node.sess.close()
