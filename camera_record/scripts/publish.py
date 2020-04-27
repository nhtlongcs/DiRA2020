#!/usr/bin/env python3
import os
import cv2
import sys
import rospy
import rospkg
import numpy as np

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

        self.video_path = rospy.get_param('~video_path', None)
        self.loop = rospy.get_param('~loop', False)
        image_path = rospy.get_param('~image_path', None)
        self.video_pub = rospy.Publisher('/camera/rgb/image_raw/compressed', CompressedImage, queue_size=1)

        self.rate = rospy.Rate(15)
        if self.video_path is not None:
            self.image = None
            self.video = cv2.VideoCapture(self.video_path)
        elif image_path is not None:
            self.image = cv2.imread(image_path)
        else:
            exit(1)
        self.pause = False

        rospy.loginfo('Node started')

    def publish(self):
        while not rospy.is_shutdown():
            if not self.pause:
                ret, self.frame = self.video.read()
                # self.frame = cv2.imread('/home/ubuntu/catkin_ws/src/dira/camera_record/test.png')
                if ret == False:
                    if self.loop:
                        rospy.logwarn('End of video. Loop!!')
                        self.video = cv2.VideoCapture(self.video_path)
                        continue
                    else:
                        rospy.loginfo('Stop!')
                        break
            
            frame = cv2.resize(self.frame, (320, 240))
            # frame = frame[:,:,0]

            # frame[frame > 100] = 255.0
            # frame[frame <= 100] = 0

            msg = image_to_data(frame)
            self.video_pub.publish(msg)

            cv2.imshow('VideoRecorder', frame)
            key = cv2.waitKey(15)
            if key == ord(' '):
                rospy.loginfo('Pause')
                self.pause = not self.pause
            elif key == ord('q'):
                break
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('camera_record_node', sys.argv, log_level=rospy.DEBUG)
    video = VideoPublish()
    video.publish()
    cv2.destroyAllWindows()