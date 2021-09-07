#!/usr/bin/env python

# Robotis Manipulator with FSS sensor array gripper attached

import rospy
import math
import time
from dynamixel_sdk import *
from slip_detection_and_grasp_stabilization.msg import Distance

# Declaring variables
M_PI = 3.14159265358979323846

# Control table address
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4
LEN_PRO_PROFILE_VELOCITY = 4

# Protocol version
PROTOCOL_VERSION = 2.0

###
# Default setting
DXL_ID = [1,2,3,4,5] # 5 is the gripper
joint_angle_data_collection = [180.0, 172.0, 180.0, 270.0, 220.0] # Angle for joints 1~4 and gripper
###

Num_of_DXL = len(DXL_ID)
param_goal_position = [None]*Num_of_DXL
dxl_present_position = [None]*Num_of_DXL

# Setting positions for joints 1~4 (data collection purposes)
for i in range(Num_of_DXL):
    joint_angle_data_collection[i] = int((joint_angle_data_collection[i]/360) * 4095)
    print(joint_angle_data_collection)

BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB2'

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

def dxl_init():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()

    # Enable dynamixel torque
    for i in range(Num_of_DXL):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL_ID[i])

    # Add parameter storage for present vosition value
    for i in range(Num_of_DXL):
        dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[i])
            quit()

def callback(data):
    global joint_angle_data_collection
    joint_angle_data_collection[-1] = int(float(data.Data)/360*4095)

def dxl_main():
    global joint_angle_data_collection
    rospy.init_node('Motor', anonymous=True)
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():

        rospy.Subscriber("dis_des", Distance, callback)
        # Set goal position for all joints
        for j in range(Num_of_DXL):
            param_goal_position[j] = [DXL_LOBYTE(DXL_LOWORD(joint_angle_data_collection[j])),
                                     DXL_HIBYTE(DXL_LOWORD(joint_angle_data_collection[j])),
                                     DXL_LOBYTE(DXL_HIWORD(joint_angle_data_collection[j])),
                                     DXL_HIBYTE(DXL_HIWORD(joint_angle_data_collection[j]))]

        # Add goal position parameter for all joints
        for j in range(Num_of_DXL):
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[j], param_goal_position[j])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[j])
                quit()

        # Write the goal position for all joints
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        # Syncread present position of all joints
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if data is available for all joints
        for k in range(Num_of_DXL):
            dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[k], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID[k])
                quit()

        # Get the present position data for all joints
        for k in range(Num_of_DXL):
            dxl_present_position[k] = groupSyncRead.getData(DXL_ID[k], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Printing present position data for all joints
        #for k in range(Num_of_DXL):
            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (
        #DXL_ID[k], joint_angle_data_collection[k], dxl_present_position[k]))

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_init()
        dxl_main()
    except rospy.ROSInterruptException:
        pass
