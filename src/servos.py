#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Servo

rb = redboard.RedBoard()

def callback(msg):
    if(msg.servo == 0):
        rb.servo7 = msg.value
    if(msg.servo == 1):
        rb.servo8 = msg.value
    if(msg.servo == 2):
        rb.servo9 = msg.value
    if(msg.servo == 3):
        rb.servo10 = msg.value
    if(msg.servo == 4):
        rb.servo11 = msg.value
    if(msg.servo == 5):
        rb.servo5 = msg.value
    if(msg.servo == 6):
        rb.servo6 = msg.value
    if(msg.servo == 7):
        rb.servo13 = msg.value
    if(msg.servo == 8):
        rb.servo27 = msg.value
    if(msg.servo == 9):
        rb.servo20 = msg.value
    if(msg.servo == 10):
        rb.servo21 = msg.value
    if(msg.servo == 11):
        rb.servo22 = msg.value

    rospy.loginfo("Servo: " + msg.servo + " , Value: " + msg.value)
        
def listener():
    rospy.init_node('redboard_servo_drvier', anonymous=True)
    rospy.Subscriber('/redboard/servo', Servo, callback)

    rospy.spin()

if __name__ == '__main__':
    print("RedBoard servo node listening...")
    listener()