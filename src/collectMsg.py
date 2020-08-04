#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import struct
from sensor_msgs.msg import NavSatFix

class GPS():
	def __init__(self):
		self.flag1 = False
		self.flag2 = False
		self.latitude = 0.0
		self.longitude = 0.0
		self.heading = 0.0
		self.client = mqtt.Client()
		self.pub = rospy.Publisher('/unionstrong/gpfpd', NavSatFix, queue_size=1)
		rospy.init_node("talker", anonymous=True)
	
	def on_connect(self, client, userdata, flags, rc):
		self.client.subscribe("/gps")
		self.client.subscribe("/hdt")
		print("connect succsee")

	def on_message(self, client, userdata, msg):
		#print("recevie")
		if msg.topic == "/gps":
			print("gps")
			info = struct.unpack('>dd',msg.payload)
			self.longitude = float(info[1])
			self.latitude = float(info[0])
			self.flag1 = True
			#self.flag2 = True

		if msg.topic == "/hdt":
			print("hdt")
			info = struct.unpack('>f', msg.payload)
			self.heading = float(info[0])
			self.flag2 = True
		#print(self.flag1," ",self.flag2)
		if self.flag1 and self.flag2:
			self.flag1 = False
			self.flag2 = False
			current_fix = NavSatFix()
			current_fix.latitude = self.latitude
			current_fix.longitude = self.longitude
			current_fix.position_covariance[1] = self.heading
			print("latitude ", self.latitude)
			print("longitude ", self.longitude)
			print("heading ", self.heading)
			self.pub.publish(current_fix)
			print("finish publish")


	def worker(self):
		self.client.on_connect = self.on_connect
		self.client.on_message = self.on_message
		self.client.connect("192.168.1.230", 1883, 600)
		self.client.loop_forever()

gps = GPS()
gps.worker()
