#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    print('callbacking')
    rospy.loginfo(rospy.get_caller_id() + "I heard ", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    print('init success')
    rospy.Subscriber("Heading", String, callback)
    print('callback success')
    rospy.spin()


listener()
