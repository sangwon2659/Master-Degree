#!/usr/bin/env python
import rospy
import numpy as np
from opencv_object_tracking.msg import Data_Single
from opencv_object_tracking.msg import Data_Array
sum_pitch = np.array([])
sum_updown = np.array([])

###
tolerance_pitch = 1.8
tolerance_updown = 2
###

def callback(data):
    global pub, sum_pitch, sum_updown
    pitch = data.Data[0]
    updown = data.Data[2]
    sum_pitch = np.append(sum_pitch,pitch)
    sum_updown = np.append(sum_updown, updown)
    if len(sum_pitch)>40 and len(sum_updown)>40:
        sum_pitch = np.delete(sum_pitch,[0])
	sum_updown = np.delete(sum_updown,[0])
	moving_avg_pitch = np.sum(sum_pitch)/40
        moving_avg_updown = np.sum(sum_updown)/40
        if moving_avg_pitch-pitch>tolerance_pitch or abs(moving_avg_updown-updown)>tolerance_updown:
            pub.publish(1.0)
        else:
            pub.publish(0.0)
    else:
        rospy.loginfo("Collecting data for moving average computation")

def talker():
    global pub
    rospy.init_node('aruco_pose_ground_truth', anonymous=True)
    pub = rospy.Publisher('aruco_pose_ground_truth', Data_Single, queue_size=10)
    rospy.Subscriber("aruco_pose", Data_Array, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
