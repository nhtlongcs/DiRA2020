#!/usr/bin/env python2
import os
import time
import cv2
import numpy as np
import tensorflow as tf
import keras
from keras.applications.mobilenet_v2 import MobileNetV2
from keras.models import *
from keras.layers import *
from keras.optimizers import *

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from keras.backend.tensorflow_backend import set_session


config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.log_device_placement = True

###
config.gpu_options.per_process_gpu_memory_fraction = 0.4

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

def unet(input_size = (224,224,3)):

    model = MobileNetV2(include_top = False, input_shape  = input_size)
    
    conv1 = Conv2DTranspose(320, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(model.layers[-1].output)
    con1 = concatenate([model.get_layer('block_16_project_BN').output, conv1], axis=3)
    conv2 = Conv2DTranspose(320, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(con1)
    conv3 = Conv2DTranspose(192, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv2)

    up1 = UpSampling2D(size = (2,2))(conv3)
    conv4 = Conv2DTranspose(192, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(up1)
    con2 = concatenate([model.get_layer("block_12_add").output, model.get_layer("block_11_add").output, conv4], axis=3)
    conv5 = Conv2DTranspose(192, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(con2)
    conv6 = Conv2DTranspose(64, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv5)

    up2 = UpSampling2D(size = (2,2))(conv6)
    conv7 = Conv2DTranspose(64, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(up2)
    con3 = concatenate([model.get_layer("block_5_add").output, model.get_layer("block_4_add").output, conv7], axis=3)
    conv8 = Conv2DTranspose(64, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(con3)
    conv9 = Conv2DTranspose(32, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv8)
    

    up3 = UpSampling2D(size = (2,2))(conv9)
    conv10 = Conv2DTranspose(24, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(up3)
    con4 = concatenate([model.get_layer("block_2_add").output,  conv10], axis=3)
    conv11 = Conv2DTranspose(24, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(con4)
    conv12 = Conv2DTranspose(16, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv11)

    up4 = UpSampling2D(size = (2,2))(conv12)
    conv13 = Conv2DTranspose(16, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(up4)
    con5 = concatenate([model.get_layer("expanded_conv_project_BN").output,  conv13], axis=3)
    conv14 = Conv2DTranspose(16, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(con5)
    conv15 = Conv2DTranspose(8, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv14)

    up5 = UpSampling2D(size = (2,2))(conv15)
    conv16 = Conv2DTranspose(4, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(up5)
    conv17 = Conv2DTranspose(2, 3, activation = 'relu', padding='same', kernel_initializer='he_normal')(conv16)
    conv18 = Conv2D(1, 1, activation = 'sigmoid', name ='output1')(conv17)

    model2 = Model(input = model.layers[0].input, output = conv18 )

    model2.compile(optimizer = Adam(lr = 1e-4), loss = 'binary_crossentropy', metrics = ['accuracy'])
    return model2

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
        # self.model = load_model(self.weight_path)
        self.model = unet()
        self.model.load_weights(self.weight_path)
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
