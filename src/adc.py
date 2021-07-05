#!/usr/bin/env python3
from time import sleep

import rospy
from sensor_msgs.msg import BatteryState

import board
import adafruit_ina260
 
i2c = board.I2C()
ina260 = adafruit_ina260.INA260(i2c)

pub = rospy.Publisher('power', BatteryState, queue_size=10)
rospy.init_node('robot_power_node', anonymous=True)

if __name__ == '__main__':
    try:
        # Loop until disconnected
        while True:
            # Create message
            msg = BatteryState()
            msg.voltage = ina260.voltage
            msg.current = ina260.current
            msg.location = "MFP"
            rospy.loginfo(msg)
            pub.publish(msg)

            sleep(1)
        
        print("Exiting, controller disconnected.")

    except rospy.ROSInterruptException:
        pass
