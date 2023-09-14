#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32

def callback(data, pub, q):
    result = data.data / q
    pub.publish(result)

def subscriber_node():
    rospy.init_node('subscriber', anonymous=True)
    pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
    rate = rospy.Rate(20)
    q = 0.15

    def wrapper_callback(data):
        callback(data, pub, q)

    sub = rospy.Subscriber('Eriksson', Int32, wrapper_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass
