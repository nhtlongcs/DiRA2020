#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool, String

pub_lcd = rospy.Publisher('/lcd_print', String, queue_size=5)

btn_state = 0

def callback_bt1(data):
    global btn_state
    if data.data:
        btn_state |= 1
    else:
        btn_state &= ~1

def callback_bt2(data):
    global btn_state
    if data.data:
        btn_state |= 2
    else:
        btn_state &= ~2

def callback_bt3(data):
    global btn_state
    if data.data:
        btn_state |= 4
    else:
        btn_state &= ~4

def callback_bt4(data):
    global btn_state
    if data.data:
        btn_state |= 8
    else:
        btn_state &= ~8

if __name__ == '__main__':
    rospy.init_node('button_control_node')

    sub_bt1 = rospy.Subscriber('/bt1_status', Bool, callback_bt1, queue_size=5)
    sub_bt2 = rospy.Subscriber('/bt3_status', Bool, callback_bt2, queue_size=5)
    sub_bt3 = rospy.Subscriber('/bt2_status', Bool, callback_bt3, queue_size=5)
    sub_bt4 = rospy.Subscriber('/bt4_status', Bool, callback_bt4, queue_size=5)

    x = 0
    y = 0

    pub_data = String()
    while not rospy.is_shutdown():

        if btn_state == 1:
            pub_data.data = '{}:{}:Button 1'.format(x,y)
        if btn_state == 2:
            pub_data.data = '{}:{}:Button 2'.format(x,y)
        if btn_state == 4:
            pub_data.data = '{}:{}:Button 3'.format(x,y)
        if btn_state == 8:
            pub_data.data = '{}:{}:Button 4'.format(x,y)

        if btn_state == 0:
            pub_data.data = '{}:{}:        '.format(x,y)

        pub_lcd.publish(pub_data)