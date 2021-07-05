#!/usr/bin/env python3

import rospy
import redboard
import math
from time import sleep 
from datetime import datetime
from datetime import timedelta
import sys

from sensor_msgs.msg import Joy

expander = redboard.PCA9685(address=0x40)
expander.frequency = 50

axesDict = {
    "lx": 0,
    "ly": 1,
    "lz": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
    "encoder": 6 }

buttonsDict = {
    "S1": 0,
    "S2": 1,
    "S3": 4,
    "ToggleDown": 2,
    "ToggleUp": 3,
    "Encoder": 5,
    "Trigger": 6,
    "LeftStick": 7,
    "RightStick": 8 }

# Initialise the arm position array
defaultPositions = [ 0.00,  0.40,  0.00, -0.80,  0.00,  0.20,  0.00,  0.20,
                     0.00, -0.40,  0.00,  0.80,  0.00, -0.20,  0.00,  0.80]

positions = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

minPositions = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  -1.0, -1.0,
                -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  -1.0, -1.0]

maxPositions = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,
                 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0]

# Index of the position to update, value is the amount to change by.
# Sets the new value if not out of bounds.
def SetPosition(index, value):
    global positions
    newValue = positions[index] + value

    if(newValue > maxPositions[index]):
        newValue = maxPositions[index]
    elif(newValue < minPositions[index]):
        newValue = minPositions[index]

    positions[index] = newValue

# Set the position of each servo
def SetServos():
    expander.servo0 = positions[0]
    expander.servo1 = positions[1]
    expander.servo2 = positions[2]
    expander.servo3 = positions[3]
    expander.servo4 = positions[4]
    expander.servo5 = positions[5]
    expander.servo6 = positions[6]
    expander.servo7 = positions[7]
    expander.servo8 = positions[8]
    expander.servo9 = positions[9]
    expander.servo10 = positions[10]
    expander.servo11 = positions[11]
    expander.servo12 = positions[12]
    expander.servo13 = positions[13]
    expander.servo14 = positions[14]
    expander.servo15 = positions[15]

#  Set all positions to None then call SetServo
def DisableServos():
    global positions
    for i in range(16):
        positions[i] = None

    SetServos()

def initialise():
    global positions
    #  Set the servos to the default positions, do this with delays to prevent overload
    #  Order of initialisation:
    #  elbow (3), 0.4 then to 0.9
    expander.servo3 = positions[3] = defaultPositions[3]
    expander.servo11 = positions[11] = defaultPositions[11]

    sleep(0.25)

    #  elbow_rotate(4) to hand(7) to 0
    expander.servo4 = positions[4] = defaultPositions[4]
    expander.servo5 = positions[5] = defaultPositions[5]
    expander.servo6 = positions[6] = defaultPositions[6]
    expander.servo7 = positions[7] = defaultPositions[7]

    expander.servo12 = positions[12] = defaultPositions[12]
    expander.servo13 = positions[13] = defaultPositions[13]
    expander.servo14 = positions[14] = defaultPositions[14]
    expander.servo15 = positions[15] = defaultPositions[15]

    sleep(0.25)

    #  flappy(0) to 0
    expander.servo0 = positions[0] = defaultPositions[0]
    expander.servo8 = positions[8] = defaultPositions[8]
    sleep(0.25)

    #  upper_rotate(2) to 0.2
    expander.servo2 = positions[2] = defaultPositions[2]
    expander.servo10 = positions[10] = defaultPositions[10]
    sleep(0.25)

    #  shoulder_foreaft(1) to -0.8
    expander.servo1 = positions[1] = defaultPositions[1]
    expander.servo9 = positions[9] = defaultPositions[9]

# Rotation is centred on 0.5 so need to map for that
def MapAxis(input):
    if((input >= 0.43) & (input <= 0.57)):
        return 0
    else:
        return input - 0.5

def ReduceAxis(input):
    return input / 25

# Print the values of all positions to 3 decimal points
def PrintPositions():    
    print(f'{positions[0]:.2f}, {positions[1]:.2f}, {positions[2]:.2f}, {positions[3]:.2f}, {positions[4]:.2f}, {positions[5]:.2f}, {positions[6]:.2f}, {positions[7]:.2f}, {positions[8]:.2f}, {positions[9]:.2f}, {positions[10]:.2f}, {positions[11]:.2f}, {positions[12]:.2f}, {positions[13]:.2f}, {positions[14]:.2f}, {positions[15]:.2f}')

def callback(data):
    if(data.buttons[buttonsDict["Encoder"]] == 1):
        DisableServos()
        sys.exit(0)

    # buttons[3] is the toggle button up on the controller
    if(data.buttons[buttonsDict["ToggleUp"]] == 1):
        global positions, leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        if(data.buttons[buttonsDict["Trigger"]] == 1):
            PrintPositions()

        lx = data.axes[axesDict["lx"]]
        ly = data.axes[axesDict["ly"]]
        lz = data.axes[axesDict["lz"]]

        rx = data.axes[axesDict["rx"]]
        ry = data.axes[axesDict["ry"]]
        rz = data.axes[axesDict["rz"]]

        # if left stick pressed 
        if(data.buttons[buttonsDict["LeftStick"]] == 0):
            SetPosition(1, ReduceAxis(ly)) 
            SetPosition(2, ReduceAxis(lx))        

            # leftRotate = MapAxis(lz)
            # if(leftRotate != 0):
            SetPosition(0, -ReduceAxis(lz))
        else:
            # S2 = 0 (arm control), S2 = 1 (hand control)
            if(data.buttons[buttonsDict["S2"]] == 0):
                SetPosition(3, ReduceAxis(ly)) 
                SetPosition(2, -ReduceAxis(lx))
                
                # leftRotate = MapAxis(lz)
                # if(leftRotate != 0):
                SetPosition(4, ReduceAxis(lz))
            else:
                SetPosition(5, ReduceAxis(ly)) 
                SetPosition(6, -ReduceAxis(lx))
                
                # leftRotate = MapAxis(lz)
                # if(leftRotate != 0):
                SetPosition(7, -ReduceAxis(lz))

        #  if right button pressed, control the elbow/hand instead
        if(data.buttons[buttonsDict["RightStick"]] == 0):
            SetPosition(9, -ReduceAxis(ry))
            SetPosition(10, ReduceAxis(rx))
            
            # rightRotate = MapAxis(rz)
            # if(rightRotate != 0):
            SetPosition(8, -ReduceAxis(rz))
        else:
            # S3 = 0 (arm control), S3 = 1 (hand control)
            if(data.buttons[buttonsDict["S3"]] == 0):
                SetPosition(11, -ReduceAxis(ry))
                SetPosition(10, -ReduceAxis(rx))

                # rightRotate = MapAxis(rz)
                # if(rightRotate != 0):
                SetPosition(12, ReduceAxis(rz))
            else:
                SetPosition(13, -ReduceAxis(ry))
                SetPosition(14, -ReduceAxis(rx))

                # rightRotate = MapAxis(rz)
                # if(rightRotate != 0):
                SetPosition(15, ReduceAxis(rz))

        SetServos()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('arm_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Arm node listening...")
    initialise()

    sleep(5)
    
    listener()
