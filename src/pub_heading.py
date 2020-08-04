#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import struct
from std_msgs.msg import Float64


pub = rospy.Publisher("Heading", Float64, queue_size=10)
rospy.init_node("talker1", anonymous=True)
rate = rospy.Rate(5)

def on_connect(client, userdata, flags, rc):
    client.subscribe("/bat")
    print("connect success")

def on_message(client, userdata, msg):  
    if msg.topic == "/bat":
        heading = struct.unpack('>B',msg.payload)
        print(heading)
        rate.sleep()

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.1.230", 1883, 600)
client.loop_forever()
#receive_pose(client)
