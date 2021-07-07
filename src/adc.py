#!/usr/bin/env python3
import rospy
import redboard
from time import sleep
from rosredboard.msg import ADC

pub = rospy.Publisher('adc', ADC, queue_size=10)
rospy.init_node('adc_node', anonymous=True)

rb = redboard.RedBoard()

if __name__ == '__main__':
    try:
        # Loop until disconnected
        while not rospy.is_shutdown():
            # Create message
            msg = ADC()
            msg.adc0 = rb.adc0
            msg.adc1 = rb.adc1
            msg.adc2 = rb.adc2
            msg.adc3 = rb.adc3

            # rospy.loginfo(msg)
            pub.publish(msg)

            sleep(1)

    except rospy.ROSInterruptException:
        pass
