#!/usr/bin/env python2
import os
import time
import cv2
import numpy as np
import tensorflow as tf
import keras
from keras.models import *

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from keras.backend.tensorflow_backend import set_session


config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.log_device_placement = True

sess = tf.Session(config=config)
set_session(sess)

# global model
# model = load_model("/home/wan/catkin_ws/src/mobilenet/model/model.h5")
# model.summary()
# global graph
# graph = tf.get_default_graph()

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

        self.bridge = CvBridge()

        self.init_model()
        self.sub_image = rospy.Subscriber(
            self.image_topic, CompressedImage, self.img_callback, queue_size=1)  # , buff_size=4294967296)
        self.pub_lane = rospy.Publisher('/lane_detect/lane_seg/compressed', CompressedImage, queue_size=1)

    def init_model(self):
        self.model = load_model(self.weight_path)
        self.model.summary()
        self.graph = tf.get_default_graph()

    def img_callback(self, data):
        # print('check')
        with self.graph.as_default():
            try:
                cv_image = data_to_image(data.data)
            except:
                print('Cannot convert to cv_image')
            # cv2.imshow('Camera', cv_image)
            img = cv2.resize(cv_image, (224, 224))
            img = img.squeeze() / 255.
            img = np.expand_dims(img, 0)
            seg = self.model.predict(img)
            dst = cv2.resize(seg[0], (320, 240))
            dst = dst.squeeze() * 255.

            # lane_seg = self.bridge.cv2_to_imgmsg(dst)
            lane_seg = image_to_data(dst)
            self.pub_lane.publish(lane_seg)

            # cv2.imshow('Binary', dst)
            cv2.waitKey(1)


if __name__ == "__main__":
    # cv2.namedWindow('Camera')
    # cv2.namedWindow('Binary')
    rospy.init_node('mobilenet_node')
    mobilenet_node()
    rospy.spin()
