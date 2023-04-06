#!/usr/bin/env python
"""
#title           :ps4msd.py
#description     :Python Script to read PS4 Joystick with serial communication protocol
#author          :Achmad Syahrul Irwansyah
#date            :2023/03/31
#version         :0.1
#usage           :Python
#notes           :
#python_version  :2.7
#==============================================================================
"""

import rospy
import serial

from geometry_msgs.msg import Twist
from ps4msd.msg import Joystate

parsed = []
joystate = Joystate()
twist = Twist()

def read_serial(serial_port, baudrate, speed, turn):
    global parsed
    # setup serial connection
    try:
        joystick=serial.Serial(serial_port,baudrate,timeout=1)
        joystick.baudrate=baudrate
        joystick.reset_input_buffer()
        cond = "reading serial"
        print(cond)
    except:
        cond = "trouble: init. USB"
        print(cond)

    max_speed = speed
    max_turn = turn

    while not rospy.is_shutdown():
        # read serial line and decode it into variables
        line_joystick=joystick.readline().split(",")
        parsed = [x.rstrip() for x in line_joystick]
        
        try:
            pub_joystate()
            pub_twist(speed, turn)

            [speed, turn] = set_vel(speed, turn, max_speed, max_turn)

        except BaseException as e:
            print(e)

def pub_joystate():
    global parsed
    joystate_pub = rospy.Publisher("/joystick/joystate",Joystate,queue_size=10)

    #Left Analog Hat
    joystate.left_analog_y = 15 - int(("0x"+parsed[3]),16)%16
    joystate.left_analog_x = (int(("0x"+parsed[3]),16)-int(("0x"+parsed[3]),16)%16)/16
    
    #Right Analog Hat
    joystate.right_analog_y = 15 - int(("0x"+parsed[4]),16)%16
    joystate.right_analog_x = (int(("0x"+parsed[4]),16)-int(("0x"+parsed[4]),16)%16)/16

    #Analog Button
    joystate.r2_analog = int(("0x"+parsed[5]),16)%16
    joystate.l2_analog = (int(("0x"+parsed[5]),16)-int(("0x"+parsed[5]),16)%16)/16

    #Press Button
    #Binary value which indicates [R3, R1, Square, Cross, Circle, Triangle] consecutively
    right_button = [int(x) for x in bin(int(("0x"+parsed[6]),16))[2:].zfill(6)]
    #Binary value which indicates [L3, L1, Left, Down, Right, Up] consecutively
    left_button = [int(x) for x in bin(int(("0x"+parsed[7]),16))[2:].zfill(6)]

    joystate.button_triangle = right_button[5]
    joystate.button_circle = right_button[4]
    joystate.button_cross = right_button[3]
    joystate.button_square = right_button[2]
    joystate.button_r1 = right_button[1]    
    joystate.button_r3 = right_button[0]

    joystate.button_up = left_button[5]
    joystate.button_right = left_button[4]
    joystate.button_down = left_button[3]
    joystate.button_left = left_button[2]
    joystate.button_l1 = left_button[1]    
    joystate.button_l3 = left_button[0]

    joystate_pub.publish(joystate)

def pub_twist(speed,turn):
    global joystate, twist
    twist_pub = rospy.Publisher("/joystick/cmd_vel",Twist,queue_size=10)

    linear = float(joystate.right_analog_y/15.0)*2.0*speed - speed
    angular = float((15-joystate.right_analog_x)/15.0)*2.0*turn - turn

    twist.linear.x = linear
    twist.angular.z = angular

    twist_pub.publish(twist)

def set_vel(speed,turn,max_speed,max_turn):
    global joystate
    if (joystate.button_r1 == 1):
        if (joystate.button_up == 1):
            speed = speed + 0.01
        elif (joystate.button_down == 1):
            speed = speed - 0.01
        elif (joystate.button_right == 1):
            turn = turn + 0.01
        elif (joystate.button_left == 1):
            turn = turn - 0.01
        else:
            pass
    else:
        pass

    speed = min(max(speed, 0), max_speed)
    turn = min(max(turn, 0), max_turn)

    return speed, turn

if __name__ == "__main__":
    try:
        rospy.init_node('joystick')
    
        serial_port = rospy.get_param("~serial_port","/dev/ttyACM0")
        baudrate = rospy.get_param("~baudrate",115200)
        max_speed = rospy.get_param("~max_speed",0.4)
        max_turn = rospy.get_param("~max_turn",0.8)
        
        rate = rospy.Rate(100) # 100 Hz
        read_serial(serial_port,baudrate,max_speed,max_turn)
        rate.sleep()

    except rospy.ROSInterruptException:
        pass
