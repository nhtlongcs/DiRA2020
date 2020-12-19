#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

import sys, select, termios, tty

KEY_UP = 'w'
KEY_DOWN = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_QUIT = 'q'

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(key)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("car_control_console_node")

    pub_angle = rospy.Publisher('/set_angle', Float32, queue_size=5)
    pub_speed = rospy.Publisher('/set_speed', Float32, queue_size=5)

    angle = Float32()
    speed = Float32()

    try:
        while not rospy.is_shutdown():

            key = getKey()

            if key == KEY_QUIT:
                break
            
            angle.data = 0

            if key == KEY_UP:
                speed.data = 15
            elif key == KEY_DOWN:
                speed.data = 0
            elif key == KEY_LEFT:
                angle.data = 40
            elif key == KEY_RIGHT:
                angle.data = -40

            pub_angle.publish(angle)
            pub_speed.publish(speed)

    except Exception as e:
        print(e)
    finally:
        angle = Float32()
        speed = Float32()

        pub_angle.publish(angle)
        pub_speed.publish(speed)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

