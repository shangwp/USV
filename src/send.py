#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import struct
from std_msgs.msg import String
def sendPose(client):
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)

    topic = '/ctrl'
    v = 0.5
    r = 0
    p = 50
    data = struct.pack(">ffB", v, r, p)

    while not rospy.is_shutdown():
        client.publish(topic, payload=data)
        print("message send")
        #client.loop_forever()
        rate.sleep()


client = mqtt.Client()
client.connect("192.168.1.230", 1883, 600)
sendPose(client)

