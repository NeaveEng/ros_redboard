#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Motors

rb = redboard.RedBoard()

rb.m0_invert = False
rb.m1_invert = False

def callback(msg):
    rb.m0 = msg.m0
    rb.m1 = msg.m1

    rospy.loginfo("M0: " + msg.m0 + " , M1: " + msg.m1)
        
def listener():
    rospy.init_node('redboard_motor_drvier', anonymous=True)
    rospy.Subscriber('/redboard/motors', Motors, callback)

    rospy.spin()

if __name__ == '__main__':
    print("RedBoard motor node listening...")
    listener()