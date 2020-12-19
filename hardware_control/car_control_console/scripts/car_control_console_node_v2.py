#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

import sys, select, termios, tty, fcntl, os

from threading import Thread
from queue import Queue, Empty

KEY_UP = 'w'
KEY_DOWN = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_QUIT = 'q'

pub_angle = rospy.Publisher('/set_angle', Float32, queue_size=1)
pub_speed = rospy.Publisher('/set_speed', Float32, queue_size=1)

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key

def get_input(threadname, q_ud, q_lr):

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == KEY_UP or key == KEY_DOWN or key == KEY_QUIT:
                q_ud.put(key)

            if key == KEY_LEFT or key == KEY_RIGHT or key == KEY_QUIT:
                q_lr.put(key)

            if key == KEY_QUIT:
                return
    except Exception as e:
        print(e)

def get_input_ud(threadname, queue):

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == KEY_UP or key == KEY_DOWN or key == KEY_QUIT:
                queue.put(key)

            if key == KEY_QUIT:
                return
    except Exception as e:
        print(e)

def get_input_lr(threadname, queue):

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == KEY_LEFT or key == KEY_RIGHT or key == KEY_QUIT:
                queue.put(key)

            if key == KEY_QUIT:
                return
    except Exception as e:
        print(e)

def job_ud(threadname, queue):

    speed = Float32()
    try:
        while not rospy.is_shutdown():
            speed.data = 0
            try:
                key = queue.get(block=False)

                if key == KEY_QUIT:
                    speed.data = 0
                    pub_speed.publish(speed)
                    return

                if key == KEY_UP:
                    speed.data = 15
                elif key == KEY_DOWN:
                    speed.data = 0

            except Empty:
                pass
            pub_speed.publish(speed)
            
    except Exception as e:
        print(e)
    finally:
        pub_speed.publish(speed)

def job_lr(threadname, queue):

    angle = Float32()
    try:
        while not rospy.is_shutdown():
            key = queue.get()

            # print(key)

            angle.data = 0

            if key == KEY_QUIT:
                angle.data = 0
                pub_angle.publish(angle)
                return

            if key == KEY_LEFT:
                angle.data = 40
            elif key == KEY_RIGHT:
                angle.data = -40

            pub_angle.publish(angle)
    except Exception as e:
        print(e)
    finally:
        pub_angle.publish(angle)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("car_control_console_node")

    queue_ud = Queue(maxsize=1)
    queue_lr = Queue(maxsize=1)

    # thread_0 = Thread(target=get_input_ud, args=('0', queue_ud))
    # thread_1 = Thread(target=get_input_lr, args=('1', queue_lr))
    thread_1 = Thread(target=get_input, args=('1', queue_ud, queue_lr))
    thread_2 = Thread(target=job_ud, args=('2', queue_ud))
    # thread_3 = Thread(target=job_lr, args=('3', queue_lr))

    # thread_0.start()
    thread_1.start()
    thread_2.start()
    # thread_3.start()

    # thread_0.join()
    thread_1.join()
    thread_2.join()
    # thread_3.join()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

