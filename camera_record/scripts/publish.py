#!/usr/bin/env python2
import os
import cv2
import sys
import rospy
import rospkg
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

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


class VideoPublish:
    def __init__(self):
        self.rgb_frame_count = 0
        self.depth_frame_count = 0

        video_path = rospy.get_param('~video_path')
        self.video_pub = rospy.Publisher('/camera/rgb/image_raw/compressed', CompressedImage, queue_size=1)
        rospy.loginfo('Node started')
    
        self.rate = rospy.Rate(15)
        self.video = cv2.VideoCapture(video_path)
        self.pause = False

    def publish(self):
        while not rospy.is_shutdown():
            if not self.pause:
                ret, self.frame = self.video.read()
                if ret == False:
                    rospy.logerr('Could not read frame. Stop!')
                    exit(0)
            
            frame = cv2.resize(self.frame, (320, 240))
            frame = frame[:,:,0]

            frame[frame > 100] = 255.0
            frame[frame <= 100] = 0

            msg = image_to_data(frame)
            self.video_pub.publish(msg)

            cv2.imshow('temp', frame)
            key = cv2.waitKey(1)
            if key == ord(' '):
                rospy.loginfo('Pause')
                self.pause = not self.pause
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('camera_record_node', sys.argv, log_level=rospy.DEBUG)
    video = VideoPublish()
    video.publish()
    cv2.destroyAllWindows()