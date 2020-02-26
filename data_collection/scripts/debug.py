#!/usr/bin/env python2
import os
import cv2
import rospy
import rospkg
import sys

if __name__ == "__main__":
    rospy.init_node('debug_node', sys.argv)

    rospack = rospkg.RosPack()
    rospack.list()
    pkg_path = rospack.get_path('data_collection')
    print('Package path: %s' % pkg_path)

    # classname = rospy.get_param('~class')
    # if classname not in ['forward', 'left', 'right', 'notleft', 'notright', 'stop']:
    #     rospy.logfatal('Classname should be one of {}'.format(classname))
    classname = 'left'

    file_box = os.path.join(pkg_path, 'images', classname, 'bounding_box.csv')
    with open(file_box, 'r') as f:
        lines = f.readlines()

    lines = [line.rstrip().split(',') for line in lines]
    print(lines)

    for line in lines:
        filepath, x, y, w, h = line
        x, y, w, h = map(int, line[1:])
        image = cv2.imread(filepath, cv2.IMREAD_ANYCOLOR)
        image = cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.imshow('Frame', image)
        key = cv2.waitKey(0)
        if key == ord('q'):
            exit(0)

    cv2.destroyAllWindows()