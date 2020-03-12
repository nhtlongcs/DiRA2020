#!/usr/bin/env python2
import os
import cv2
import sys
import rospy
import rospkg
from datetime import datetime

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

def data_to_image(data):
    np_arr = np.fromstring(data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np


class VideoRecord:
    def __init__(self):
        self.rgb_frame_count = 0
        self.depth_frame_count = 0

        rospack = rospkg.RosPack()
        rospack.list()
        pkg_path = rospack.get_path('camera_record')

        use_sim = rospy.get_param('~use_sim')
        rgb_topic = rospy.get_param('~rgb_topic')
        depth_topic = rospy.get_param('~depth_topic')

        
        if use_sim:
            self.rgb_sub = rospy.Subscriber(rgb_topic, CompressedImage, self.rgb_callback, queue_size=1, buff_size=2**24)
            self.depth_sub = rospy.Subscriber(depth_topic, CompressedImage, self.depth_sim_callback, queue_size=1, buff_size=2**24)
        else:
            self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback, queue_size=1, buff_size=2**24)
            self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_cam_callback, queue_size=1, buff_size=2**24)
            
        now = datetime.now().strftime('%D-%M-%Y_%H-%M-%S')
        self.save_dir = rospy.get_param('~save_dir', os.path.join(pkg_path, 'record', now))
        os.makedirs(self.save_dir)
        rospy.loginfo('Save images to {}'.format(self.save_dir))
        rospy.loginfo('Node started')

    def save_image(self, filepath, data):
        try:
            cv_image = data_to_image(data.data)
            if not cv2.imwrite(filepath, cv_image):
                rospy.logerr('Cannot save frame to {}'.format(filepath))
        except:
            rospy.logerr('Cannot convert to cv_image')
            return

    def rgb_callback(self, data):
        self.rgb_frame_count += 1
        rospy.logdebug('RGBCallback %d' % self.rgb_frame_count)
        filepath = os.path.join(self.save_dir, 'rgb_' + str(self.rgb_frame_count) + '.jpg')
        self.save_image(filepath, data)

    def depth_sim_callback(self, data):
        self.depth_frame_count += 1
        rospy.logdebug('SimDepthCallback %d' % self.depth_frame_count)
        filepath = os.path.join(self.save_dir, 'depth_' + str(self.depth_frame_count) + '.tiff')
        self.save_image(filepath, data)

    def depth_cam_callback(self, data):
        self.depth_frame_count += 1
        rospy.logdebug('CamDepthCallback %d' % self.depth_frame_count)
        filepath = os.path.join(self.save_dir, 'depth_' + str(self.depth_frame_count) + '.tiff')
        self.save_image(filepath, data)

if __name__ == "__main__":
    rospy.init_node('camera_record_node', sys.argv, log_level=rospy.DEBUG)
    VideoRecord()
    rospy.spin()