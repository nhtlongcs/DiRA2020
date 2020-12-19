#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32, Bool

import sys, termios, tty

from pynput import keyboard

import math

KEY_FORWARD = keyboard.Key.up
KEY_BACKWARD = keyboard.Key.down
KEY_LEFT = keyboard.Key.left
KEY_RIGHT = keyboard.Key.right
KEY_QUIT = keyboard.Key.esc
KEY_BRAKE = keyboard.Key.space

KEYS = [
    KEY_FORWARD,
    KEY_BACKWARD,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_QUIT,
    KEY_BRAKE
]

key_state = {
    KEY_QUIT: False,
    KEY_FORWARD: False,
    KEY_BACKWARD: False,
    KEY_LEFT: False,
    KEY_RIGHT: False,
    KEY_BRAKE: False
}

pub_angle = rospy.Publisher('/set_angle', Float32, queue_size=1)
pub_speed = rospy.Publisher('/set_speed', Float32, queue_size=1)
speed = Float32()
angle = Float32()
is_brake_ss = False

def callback_ss(data):
    global key_state
    global speed_t
    if not data.data:
        key_state[KEY_BACKWARD] = True
        speed_t = 70
        speed.data = speed_t
        pub_speed.publish(speed)

sub_ss = rospy.Subscriber('/ss2_status', Bool, callback_ss, queue_size=5)

def on_press(key):
    global key_state
    global speed_t
    try:
        key = key.char
    except:
        pass

    if key not in KEYS:
        return

    if key_state[key] == False:
        key_state[key] = True

        if key == KEY_FORWARD:
            if speed_t >= 70:
                speed_t = 0
            speed_t += 10
        if key == KEY_BACKWARD:
            speed_t = 70
        if key == KEY_LEFT:
            angle.data = 35
        if key == KEY_RIGHT:
            angle.data = -35

        speed.data = speed_t
        pub_angle.publish(angle)
        pub_speed.publish(speed)


def on_release(key):
    global key_state

    try:
        key = key.char
    except:
        pass

    if key not in KEYS:
        return


    if key_state[key] == True:
        key_state[key] = False
        
        if key == KEY_LEFT:
            angle.data = 0
        if key == KEY_RIGHT:
            angle.data = 0

        pub_angle.publish(angle)

if __name__ == "__main__":
    
    rospy.init_node("car_control_console_node")

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    )
    listener.start()

    # rate = rospy.Rate(40)



    # settings = termios.tcgetattr(sys.stdin)
    # tty.setraw(sys.stdin.fileno())

    speed_t = 0
    rospy.spin()

    # while not rospy.is_shutdown():
    #     rate.sleep()
    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
