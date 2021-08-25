#!/usr/bin/env python

#Subscribes to the desired distance
# Controls the motor with respect to the subscribing data

import rospy
import math
import time
from dynamixel_sdk import *
from gripperslip.msg import Distance

# Declaring variables
subscribed_data = 0

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting
DXL_ID                      = 4                   # Dynamixel#1 ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def callback(data):
    global subscribed_data
    subscribed_data = int(data.Data)

def dxl():
    global subscribed_data
    rospy.init_node('Motor', anonymous=True)
    rate = rospy.Rate(0.5)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    subscribed_data = dxl_present_position

    while not rospy.is_shutdown():
        
    	# Subscribing desired distance data     
        rospy.Subscriber("dis_des", Distance, callback)

        # Acquiring current information of the motor
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    	
	if subscribed_data == 20000 and desired_position>0 and desired_position<1304:
		desired_position-=1
        	dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, desired_position)
		print("Desired Position: {} Current Position: {}".format(desired_position, dxl_present_position))	

	if subscribed_data>0 and subscribed_data<1304:
		desired_position=subscribed_data
        	dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, subscribed_data)
		print("Current Position: {}".format(dxl_present_position))	
        rate.sleep()

if __name__ == '__main__':
    try:
        dxl()
    except rospy.ROSInterruptException:
        pass
