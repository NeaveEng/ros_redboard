#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Servo

#  Get the expander address
expander_address = rospy.get_param('/expander_address', 64)
rospy.loginfo("Expander address: " + str(expander_address))

expander = redboard.PCA9685(address=expander_address)
expander.frequency = 50

def callback(msg):
    if(msg.servo == 0):
        expander.servo0 = msg.value
    if(msg.servo == 1):
        expander.servo1 = msg.value
    if(msg.servo == 2):
        expander.servo2 = msg.value
    if(msg.servo == 3):
        expander.servo3 = msg.value
    if(msg.servo == 4):
        expander.servo4 = msg.value
    if(msg.servo == 5):
        expander.servo5 = msg.value
    if(msg.servo == 6):
        expander.servo6 = msg.value
    if(msg.servo == 7):
        expander.servo7 = msg.value
    if(msg.servo == 8):
        expander.servo8 = msg.value
    if(msg.servo == 9):
        expander.servo9 = msg.value
    if(msg.servo == 10):
        expander.servo10 = msg.value
    if(msg.servo == 11):
        expander.servo11 = msg.value
    if(msg.servo == 12):
        expander.servo12 = msg.value
    if(msg.servo == 13):
        expander.servo13 = msg.value
    if(msg.servo == 14):
        expander.servo14 = msg.value
    if(msg.servo == 15):
        expander.servo15 = msg.value
        
def listener():
    rospy.init_node('redboard_servo_expander_driver', anonymous=True)
    rospy.Subscriber('/redboard/expander', Servo, callback)

    rospy.spin()

if __name__ == '__main__':
    print("RedBoard servo expander node listening...")
    listener()