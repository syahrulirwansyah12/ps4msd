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

import serial

def read_serial():
    cond = "trouble: init. USB"
    # setup serial connection
    try:
        joystick=serial.Serial("/dev/ttyACM0",115200,timeout=1)
        joystick.baudrate=115200
        joystick.reset_input_buffer()
    except:
        print(cond)

    cond = "reading serial"
    print(cond)

    while True:
        # read serial line and decode it into variables
        line_joystick=joystick.readline().split(",")
        parsed = [x.rstrip() for x in line_joystick]

        # asssign the massage's value
        #odom_msg.linear.x = float(parsed[0]) # left velocity
        #odom_msg.angular.z = float(parsed[1]) # right velocity
        #mode.data = int(parsed[2])
        #print(len(parsed))
        
        try:
            #Left Analog Hat
            LeftHatY=int(("0x"+parsed[3]),16)%16
            LeftHatX=(int(("0x"+parsed[3]),16)-int(("0x"+parsed[3]),16)%16)/16

            #Right Analog Hat
            RightHatY=int(("0x"+parsed[4]),16)%16
            RightHatX=(int(("0x"+parsed[4]),16)-int(("0x"+parsed[4]),16)%16)/16

            #Analog Button
            R2=int(("0x"+parsed[5]),16)%16
            L2=(int(("0x"+parsed[5]),16)-int(("0x"+parsed[5]),16)%16)/16

            #Press Button
            rightButton=[int(x) for x in bin(int(("0x"+parsed[6]),16))[2:].zfill(8)] #Binary value which indicates [x, x, R3, R1, Square, Cross, Circle, Triangle] consecutively
            leftButton=[int(x) for x in bin(int(("0x"+parsed[7]),16))[2:].zfill(8)] #Binary value which indicates [x, x, L3, L1, Left, Down, Right, Up] consecutively

            #print(LeftHatY, LeftHatX, RightHatY, RightHatX)
            #print(L2, R2)
            #print(rightButton)
            print(leftButton)
            #print(parsed)

        except BaseException as e:
            print(e)

if __name__ == "__main__":
    read_serial()
