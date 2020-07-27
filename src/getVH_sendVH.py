#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import struct
from std_msgs.msg import String

def callback(data, args):
    print('callbacking')
    client = args[0]
    topic = '/ctrl'
    p = 50
    v,r = struct.unpack()
    message = struct.pack('>ffB',v,r,p)
    client.publish(topic, payload=data)
    print('publish success')


def listener():
    rospy.init_node('control', anonymous=True)
    print('init success')
    client = mqtt.Client()
    client.connect("192.168.1.230",1883,600)
    rospy.Subscriber("TopicName", MsgType, callback,(client))
    print('callback success')
    rospy.spin()

listener()
