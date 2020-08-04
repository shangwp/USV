#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import paho.mqtt.client as mqtt
import struct
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class Test:
	def __init__(self):
		self.M_PI = 3.14159265
		self.coordinate = []
		self.client = mqtt.Client()
		self.client.connect("192.168.1.230",1883,600)
		self.velocity = 0.2
		#self.message = struct.pack('>ffB', self.velocity, 0, 50)
		#self.topic = '/ctrl'
		self.pub = rospy.Publisher('ctrlBoat', Vector3, queue_size=10)
		self.f = file('velocity.txt', 'a+')

	def callback(self, data):
		vE = data.position_covariance[5]
		vN = data.position_covariance[6]
		V = (vE**2 + vN**2)**0.5


		context = str(self.velocity) + '\t' + str(V) + '\n'
		self.f.write(context)
		#self.f.close()
		print("current velocity: ",V)

		ctrl = Vector3()
		ctrl.x = self.velocity
		ctrl.y = 0.0
		
		self.pub.publish(ctrl)

	def listener(self):
		rospy.init_node('test', anonymous=True)
		print('init success')
	    # 节点订阅
		rospy.Subscriber("unionstrong/gpfpd", NavSatFix, self.callback)
		print('callback success')
		rospy.spin()

	def currentVelocity(self):
		if len(self.coordinate) < 2:
			return None
		else:
			dx = self.coordinate[-1][0] - self.coordinate[-2][0]
			dy = self.coordinate[-1][1] - self.coordinate[-2][1]
			velocity = (dx**2 + dy**2)**0.5 * 20
			return velocity

test = Test()
test.listener()
