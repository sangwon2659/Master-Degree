#!/usr/bin/env python

import rospy
import getch
from std_msgs.msg import Int32

def talker():
        pub = rospy.Publisher('HCmotor', Int32, queue_size=10)
	pub2 = rospy.Publisher('HCmotor_', Int32, queue_size=10)
        rospy.init_node('HCmotor', anonymous = True)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
                x = input("Speed : ")
                rospy.loginfo(str(x))
                pub.publish(x)
		pub2.publish(x/10)

if __name__ == '__main__':
        try:
                talker()
        except rospy.ROSInterruptException:
                pass
