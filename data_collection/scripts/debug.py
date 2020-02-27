#!/usr/bin/env python2
import os
import cv2
import rospy
import rospkg
import sys

if __name__ == "__main__":
    rospy.init_node('debug_node', sys.argv, log_level=rospy.DEBUG)

    rospack = rospkg.RosPack()
    rospack.list()
    pkg_path = rospack.get_path('data_collection')
    print('Package path: %s' % pkg_path)

    # classname = rospy.get_param('~class')
    # if classname not in ['forward', 'left', 'right', 'notleft', 'notright', 'stop']:
    #     rospy.logfatal('Classname should be one of {}'.format(classname))
    classname = 'stop'

    file_box = os.path.join(pkg_path, 'images', classname, 'bounding_box.csv')
    with open(file_box, 'r') as f:
        lines = f.readlines()

    lines = [line.rstrip().split(',') for line in lines]

    i = 0
    while i < len(lines):
        line = lines[i]
        filepath, x, y, w, h = line
        x, y, w, h = map(int, line[1:])
        rospy.logdebug('File: {}'.format(filepath))
        image = cv2.imread(filepath, cv2.IMREAD_ANYCOLOR)
        image = cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.imshow('Frame', image)
        key = cv2.waitKeyEx(0)
        if key == ord('q'):
            exit(0)
        elif key == ord('a'): # LEFT
            i = max(i - 1, 0)
        elif key == ord('d'): # RIGHT
            i = min(i + 1, len(lines) - 1)

    cv2.destroyAllWindows()