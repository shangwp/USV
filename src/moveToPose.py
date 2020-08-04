#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import struct
import math
import csv
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3

class CtrlBoat:
	def __init__(self):
		self.track_point = []  
		f = open('track.csv')
		reader = csv.reader(f)
		count = 0
		for line in reader:
			if count % 3 == 0:
				x = float(line[1])*20037508.34/180
				y = math.log(math.tan(((90+float(line[0]))*math.pi/360))/(math.pi/180))
				y = y*20037508.34/180
				self.track_point.append([x,y])
			count += 1
		f.close()
		print("finish read")
		
		self.index = 0
		self.length = len(self.track_point)
		self.pub = rospy.Publisher('vtg', Vector3, queue_size=10)
		#rospy.init_node('talkToBoat', anonymous=True)

	def moveToPose(self, data):

		### 初始化部分数据 ###
		# 获得小船当前墨卡托坐标系坐标
		x = data.longitude*20037508.34/180
		y = math.log(math.tan(((90+data.latitude)*math.pi/360))/(math.pi/180))
		y = y*20037508.34/180
		current_position = [x,y]

		txtFile = open("newTrack.txt",'a+')
		context = str(x) + ',' + str(y) + '\n'
		txtFile.write(context)
		txtFile.close()
		print(current_position[0], current_position[1])
		print(self.track_point[self.index][0],self.track_point[self.index][1])
		# 获得小船当前朝向，处理为弧度
		theta = data.position_covariance[1]/180*math.pi
		# 处理为以北为0，顺时针为正，[-pi,pi]
		if theta > math.pi:
			theta = theta - 2 * math.pi
		# print("current heading", theta)
		# 判断是否更新目标点
		print(self.distance(self.track_point[self.index], current_position))
		if self.index < self.length -1:
			if self.distance(self.track_point[self.index], current_position) < 3:
				self.index += 1
				
		x_goal = self.track_point[self.index][0]
		y_goal = self.track_point[self.index][1]
		theta_goal = self.getGoalHeading()

		### 初始化部分数据 ###


		x_diff = x_goal - x
		y_diff = y_goal - y
		#rho = math.sqrt(x_diff**2 + y_diff**2)
		alpha = (math.atan(y_diff/x_diff)- theta + math.pi) % (2 * math.pi) - math.pi
		beta = (theta_goal - theta - alpha + math.pi) % (2 * math.pi) - math.pi

		ctrl_v = 0.2
		# if alpha > math.pi / 2 or alpha < -math.pi / 2:
		# 	ctrl_v = -ctrl_v
		ctrl_h = - (alpha*5/6 - beta/6) / math.pi
		if x_diff >= 0:
			ctrl_h = - ctrl_h

		self.talker(ctrl_v, ctrl_h)



	def listener(self):
	    rospy.init_node('listenToGPS', anonymous=True)
	    print('init success')
	    rospy.Subscriber("unionstrong/gpfpd", NavSatFix, self.moveToPose)
	    print('callback success')
	    rospy.spin()

	def talker(self,ctrl_v, ctrl_h):
		ctrl = Vector3()
		ctrl.x = ctrl_v
		ctrl.y = ctrl_h
		self.pub.publish(ctrl)

	def distance(self, point1, point2):
		return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

	def getGoalHeading(self):
		point1 = self.track_point[self.index]
		point2 = self.track_point[self.index+1]
		vector = [point2[0]-point1[0],point2[1]-point1[1]]
		if vector[0]>0 and vector[1]<=0:
			Dangle = math.pi + math.atan(vector[0]/vector[1])
		# 目标在第三象限
		elif vector[0]<=0 and vector[1]<0:
			Dangle = math.atan(vector[0]/vector[1]) - math.pi
		# 第二象限
		elif vector[0]<0 and vector[1]>=0:
			Dangle = math.atan(vector[0]/vector[1])
		# 第一象限
		else:
			Dangle = math.atan(vector[0]/vector[1])
		return Dangle

boat = CtrlBoat()
boat.listener()