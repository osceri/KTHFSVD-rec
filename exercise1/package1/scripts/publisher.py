#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def publisher_node():
    rospy.init_node('publisher', anonymous=True)
    pub = rospy.Publisher('Eriksson', Int32, queue_size=10)
    rate = rospy.Rate(20)
    k = 0
    n = 4

    while not rospy.is_shutdown():
        k += n
        pub.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
