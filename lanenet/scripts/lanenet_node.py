#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
# @Author  : Luo Yao
# @Modified  : AdamShan
# @Original site    : https://github.com/MaybeShewill-CV/lanenet-lane-detection
# @File    : lanenet_node.py


import time
import math
import tensorflow as tf
import numpy as np
import cv2

from lanenet_model import lanenet
from lanenet_model import lanenet_postprocess
from config import global_config

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from lane_detector.msg import Lane_Image


CFG = global_config.cfg
FRAMES_PER_PROCESS = 4

def data_to_image(data):
    np_arr = np.fromstring(data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np

class lanenet_detector():
    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic')
        self.weight_path = rospy.get_param('~weight_path')

        self.init_lanenet()
        self.bridge = CvBridge()

        self.cnt = 0
        sub_image = rospy.Subscriber(self.image_topic, CompressedImage, self.img_callback, queue_size=1)

    
    def init_lanenet(self):
        '''
        initlize the tensorflow model
        '''

        # self.input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')
        self.input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 128, 256, 3], name='input_tensor')
        phase_tensor = tf.constant('test', tf.string)
        net = lanenet.LaneNet(phase=phase_tensor, net_flag='vgg')
        self.binary_seg_ret, self.instance_seg_ret = net.inference(input_tensor=self.input_tensor, name='lanenet_model')

        # self.cluster = lanenet_cluster.LaneNetCluster()
        self.postprocessor = lanenet_postprocess.LaneNetPostProcessor()

        saver = tf.train.Saver()
        # Set sess configuration
        sess_config = tf.ConfigProto(device_count={'GPU': 1})
        # sess_config = tf.ConfigProto()
        sess_config.gpu_options.per_process_gpu_memory_fraction = CFG.TEST.GPU_MEMORY_FRACTION
        sess_config.gpu_options.allow_growth = CFG.TRAIN.TF_ALLOW_GROWTH
        sess_config.gpu_options.allocator_type = 'BFC'

        self.sess = tf.Session(config=sess_config)
        saver.restore(sess=self.sess, save_path=self.weight_path)

    
    def img_callback(self, data):
        try:
            cv_image = data_to_image(data.data)
        except e:
            print(e)

        self.cnt = (self.cnt + 1) % FRAMES_PER_PROCESS
        if self.cnt != 0:
            return
        # cv2.imshow("ss", cv_image)
        # cv2.waitKey(1)
        original_img = cv_image.copy()
        resized_image = self.preprocessing(cv_image)
        mask_image = self.inference_net(resized_image, original_img)
        cv2.imshow("mask", mask_image)
        cv2.waitKey(1)
        
    def preprocessing(self, img):
        # image = cv2.resize(img, (512, 256), interpolation=cv2.INTER_LINEAR)
        image = cv2.resize(img, (256, 128), interpolation=cv2.INTER_LINEAR)
        # cv2.namedWindow("ss")
        cv2.imshow("ss", image)
        cv2.waitKey(1)
        image = image / 127.5 - 1.0
        return image

    def inference_net(self, img, original_img):
        t_start = time.time()
        binary_seg_image, instance_seg_image = self.sess.run([self.binary_seg_ret, self.instance_seg_ret],
                                                        feed_dict={self.input_tensor: [img]})

        print(time.time() - t_start)
        cv2.imshow("Binary", np.array(binary_seg_image[0] * 255, np.uint8))
        cv2.waitKey(1)
        tmp = np.reshape(instance_seg_image, (128, 256, 4))
        cv2.imshow("Instance", tmp[:, :, (1,2,3)])
        cv2.waitKey(1)
        postprocess_result = self.postprocessor.postprocess(
            binary_seg_result=binary_seg_image[0],
            instance_seg_result=instance_seg_image[0],
            source_image=original_img
        )
        mask_image = postprocess_result['mask_image']
        mask_image = cv2.resize(mask_image, (original_img.shape[1],
                                                original_img.shape[0]),interpolation=cv2.INTER_LINEAR)
        mask_image = cv2.addWeighted(original_img, 0.6, mask_image, 5.0, 0)
        return mask_image


    def minmax_scale(self, input_arr):
        """

        :param input_arr:
        :return:
        """
        min_val = np.min(input_arr)
        max_val = np.max(input_arr)

        output_arr = (input_arr - min_val) * 255.0 / (max_val - min_val)

        return output_arr



if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node')
    lanenet_detector()
    rospy.spin()
