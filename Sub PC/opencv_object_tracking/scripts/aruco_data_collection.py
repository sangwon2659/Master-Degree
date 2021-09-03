#!/usr/bin/env python
import rospy
import numpy as np
from opencv_object_tracking.msg import Data
from opencv_object_tracking.msg import Data_Array
sum = np.array([])

###
Tolerance = 2.5
###

def callback(data):
    global pub, sum
    pitch = data.Data[1]
    sum = np.append(sum,pitch)
    if len(sum)>40:
        sum = np.delete(sum,[0])
        moving_avg = np.sum(sum)/40
        if moving_avg-pitch>Tolerance:
            pub.publish(1)
        else:
            pub.publish(0)
    else:
        rospy.loginfo("Collecting data for moving average computation")

def talker():
    global pub
    rospy.init_node('aruco_pose_ground_truth', anonymous=True)
    pub = rospy.Publisher('aruco_pose_ground_truth', Data, queue_size=10)
    rospy.Subscriber("aruco_pose", Data_Array, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
