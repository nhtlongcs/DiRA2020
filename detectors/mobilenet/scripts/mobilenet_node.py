#!/usr/bin/env python
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
# import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError


class mobilenet_node():
    def __init__(self):
        self.image_topic = rospy.get_param('/rgb_topic')
        self.output_topic = rospy.get_param('/lane_segmentation_topic')
        self.weight_path = rospy.get_param('~weight_path', None)
        self.is_use_deep = rospy.get_param('~use_deep', True)

        if self.is_use_deep:
            assert self.weight_path is not None

        self.init_model()
        if self.is_use_deep:
            self.proc_func = self.use_deep
        else:
            self.proc_func = self.use_imgproc

        self.bridge = CvBridge()

        if rospy.get_param('/transport_hint') == 'compressed':
            self.sub_image = rospy.Subscriber(
                self.image_topic, CompressedImage, self.img_callback, queue_size=1, buff_size=2**24)
            self.image2cv = self.bridge.compressed_imgmsg_to_cv2
        else:
            self.sub_image = rospy.Subscriber(
                self.image_topic, Image, self.img_callback, queue_size=1, buff_size=2**24)
            self.image2cv = self.bridge.imgmsg_to_cv2
        self.pub_lane = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.switch_service = rospy.Service('switch_mode', Trigger, self.switch_mode_callback)

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

        # out_pred = (out_pred.squeeze(0).squeeze(2) * 255).astype(np.uint8)
        out_pred = (out_pred.squeeze(0)[:,:,0] * 255).astype(np.uint8)
        return out_pred

    def use_imgproc(self, cv_image):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, cv_image = cv2.threshold(cv_image, 180, 255, cv2.THRESH_BINARY)
        return cv_image

    def img_callback(self, data):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data)
            # cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            cv_image = self.image2cv(data)
        except:
            print('Cannot convert to cv_image')
            return

        cv_image = self.proc_func(cv_image)
        cv_image = self.bridge.cv2_to_imgmsg(cv_image)
        self.pub_lane.publish(cv_image)

    def switch_mode_callback(self, req):
        self.is_use_deep = not self.is_use_deep
        if self.is_use_deep:
            self.proc_func = self.use_deep
        else:
            self.proc_func = self.use_imgproc

if __name__ == "__main__":
    rospy.init_node('mobilenet_node')
    node = mobilenet_node()
    rospy.spin()
    node.sess.close()
