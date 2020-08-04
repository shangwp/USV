#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import struct
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
#from sensor_msgs.msg import PointCloud2
#import sensor_msgs.point_cloud2 as pc2

class CtrlBoat:
	def __init__(self):
		self.M_PI = 3.14159265
		# 速度集合
		self.Vset = []
		self.pub = rospy.Publisher('ctrlBoat', Vector3, queue_size=10)
		self.kp = 0.1
		self.ki = 0.001
		self.kd = 0.001
		# 想要达到的目标速度
		self.target_velocity = 1.0
		# 发送给小船的控制指令
		self.control_velocity = 0.0
		self.control_heading = 0.0
		# 当前朝向
		self.current_heading = 0.0
		# 用于记录当前速度与目标速度之间的差值
		self.err = []
		# 目标点坐标，这里要使用墨卡托坐标系，或者使用经纬度，后面再处理
		self.goal_positon = [113.3833763,23.06898192]
		x = self.goal_positon[0]*20037508.34/180
		y = math.log(math.tan(((90+self.goal_positon[1])*self.M_PI/360))/(self.M_PI/180))
		y = y*20037508.34/180
		self.goal_positon[0] = x
		self.goal_positon[1] = y


	# 订阅节点的回调函数,每接收到一次消息，对速度和方向进行一次控制
	def ctrl_v(self, data):
		# 东向速度
		vE = data.position_covariance[5]
		# 北向速度
		vN = data.position_covariance[6]
		# 当前绝对速度
		current_velocity = (vE**2 + vN**2)**0.5
		# 将当前绝对速度记录入速度集
		self.Vset.append(current_velocity)
		# 计算当前速度与目标速度之差，并储存
		error = self.target_velocity - current_velocity
		self.err.append(error)
		# 每记录十个当前速度，进行一次速度控制
		if len(self.Vset)%10 == 0:
			print("current velocity = ",current_velocity)
			# 使用PID对控制速度进行调整
			self.control_velocity += self.kp*self.err[-1] + self.ki*sum(self.err[-3:]) + self.kd*(self.err[-1]-self.err[-2])
			#print("control velocity = ",self.control_velocity)

			# 发布指令
			self.talker()

			# 记录数据
			f = open('velocity.txt', 'a+')
			context = str(current_velocity)+'\t'+str(self.control_velocity)+'\t'+str(self.target_velocity)+'\n'
			f.write(context)
			f.close()
		# 每记录一千次速度，进行一次清理
		if len(self.Vset) > 1000:
			del self.Vset[:]

		# 用于测试避障部分

		# 正北方向顺时针[0,360]
		self.current_heading = data.position_covariance[1]
		self.current_heading = self.current_heading/180*self.M_PI
		# if self.current_heading <= 180:
		# 	self.current_heading = self.current_heading/180*self.M_PI
		# else:
		# 	self.current_heading = -(360 - self.current_heading)/180*self.M_PI
		# 处理后当前方向范围[-179,180]
		
		# 获得小船当前墨卡托坐标
		current_x = data.longitude*20037508.34/180
		current_y = math.log(math.tan(((90+data.latitude)*self.M_PI/360))/(self.M_PI/180))
		current_y = current_y*20037508.34/180
		current_position = [current_x, current_y]
		# 使用经纬度设定障碍物（这里应该用上面的公式再处理一次，从经纬度转换成墨卡托坐标系）
		# 或者直接使用墨卡托坐标系下坐标
		obstacles = [[113.3832534,23.06893298],[113.3832534,23.06892498],[113.3831805,23.06889804],[113.3831955,23.06891904],[1133833234,23.06893298],[113.3833234,23.06893798]]
		# 将障碍物坐标和目标点坐标转换到墨卡托坐标系
		for i in obstacles:
			x = i[0]*20037508.34/180
			y = math.log(math.tan(((90+i[1])*self.M_PI/360))/(self.M_PI/180))
			y = y*20037508.34/180
			i[0] = x
			i[1] = y

		# 将坐标转换到相对坐标系
		for i in range(len(obstacles)):
			obstacles[i][0] = obstacles[i][0] - current_position[0]
			obstacles[i][1] = obstacles[i][1] - current_position[1]

		# 开始判断相交
		# start就是船的位置，一直是 0 0
		start = [0.0,0.0]
		# end是目标点，转换到相对坐标系
		end = [self.goal_positon[0]-current_position[0],self.goal_positon[1]-current_position[1]]

		#用于判断是否所有障碍物都不在当前路线上
		# 用于记录与heading相交的障碍物，内部为[[x1,y1],[x2,y2]...]
		block = []
		
		for i in range(0,len(obstacles),2):
			if self.intersec(start, end, obstacles[i], obstacles[i+1]):
				block.append(obstacles[i])
				block.append(obstacles[i+1])
		if block:
			print('find obstacles')
			# minLength用于记录到船最近的障碍物的距离
			minLength = self.caculateDistance(start, block[0], block[1])
			# target 用于记录
			target = 0
			for i in range(0, len(block), 2):
				length = self.caculateDistance(start, block[i], block[i+1])
				if length < minLength:
					target = i
					minLength = length
			end = block[target]
		print("goal_positon",end[0],end[1])
		# 计算目标方向到正北方向的弧度差
		# 目标在第四象限
		if end[0]>0 and end[1]<=0:
			Dangle = self.M_PI + math.atan(end[0]/end[1])
		# 目标在第三象限
		elif end[0]<=0 and end[1]<0:
			Dangle = math.atan(end[0]/end[1]) + self.M_PI
		# 第二象限
		elif end[0]<0 and end[1]>=0:
			Dangle = 2*self.M_PI + math.atan(end[0]/end[1])
		# 第一象限
		else:
			Dangle = math.atan(end[0]/end[1])
		print("current_heading",self.current_heading)
		print("goal_heading",Dangle)

		Dangle = Dangle-self.current_heading

		if Dangle > self.M_PI:
			Dangle = Dangle - 2*self.M_PI
		if Dangle < - self.M_PI:
			Dangle = 2*self.M_PI + Dangle
		print("Dangle",Dangle)
		# 计算控制的heading，-1 ~ 1
		if Dangle > self.M_PI/2:
			Dangle = self.M_PI/2
		if Dangle < -self.M_PI/2:
			Dangle = -self.M_PI/2
		self.control_heading = Dangle/(self.M_PI/2)

		print('control heading',self.control_heading)

	# 实际情况使用的方向控制，只是多了一个读取实际障碍物的操作
	def ctrl_h(self, lidar):
		# 获得当前的xy坐标并初步确定方向
		current_x = data.longitude*20037508.34/180
		current_y = math.log(math.tan(((90+data.latitude)*self.M_PI/360))/(self.M_PI/180))
		current_y = current_y*20037508.34/180
		current_position = [current_x, current_y]

		# 获得障碍物坐标，并以点对的方式储存在obstacle里，障碍物的坐标是以小船为中心的相对坐标系
		lidar = pc2.read_points(lidar)
		points = np.array(list(lidar))
		x_points = points[:,0]
		y_points = points[:,1]
		obstacles =  [[]for i in range(len(x_points))]
	 
		count = 0
		while count<len(x_points):
			obstacles[count].append(x_points[count])
			obstacles[count].append(y_points[count])
			obstacles[count+2].append(x_points[count+1])
			obstacles[count+2].append(y_points[count+1])
			obstacles[count+3].append(x_points[count+2])
			obstacles[count+3].append(y_points[count+2])
			obstacles[count+1].append(x_points[count+3])
			obstacles[count+1].append(y_points[count+3])
			count = count + 4

		# 开始判断相交
		start = [0.0,0.0]
		end = [self.goal_positon[0]-current_position[0],self.goal_positon[1]-current_position[1]]


		# 用于记录与heading相交的障碍物，内部为[[x1,y1],[x2,y2]...]
		block = []
		
		for i in rarng(0,len(obstacles),2):
			if judgeCross(start, end, obstacles[i], obstacles[i+1]):
				block.append(obstacles[i])
				block.append(obstacles[i+1])
		if block:
			# minLength用于记录到船最近的障碍物的距离
			minLength = caculateDistance(start, block[0][0], block[0][1])
			# target 用于记录
			target = 0
			for i in range(0, len(block), 2):
				length = caculateDistance(start, block[i], block[i+1])
				if length < minLength:
					target = i
					minLength = length
			end = block[target]

		if end[0]>0 and end[1]<=0:
			Dangle = self.M_PI/2
		elif end[0]<0 and end[1]<=0:
			
			Dangle = -self.M_PI/2
		else:
			Dangle = math.atan(end[0]/end[1])
		self.control_heading = Dangle/self.M_PI
		
	def listener(self):

	    rospy.init_node('listenToGPS', anonymous=True)
	    #rospy.init_node('listenToRadar', anonymous=True)
	    print('init success')

	    rospy.Subscriber("unionstrong/gpfpd", NavSatFix, self.ctrl_v)
	    #rospy.Subscriber("/filtered_points", PointCloud2, self.ctrl_h)
	    print('callback success')
	    rospy.spin()

	# 发布速度控制指令
	def talker(self):
		ctrl = Vector3()
		ctrl.x = self.control_velocity
		ctrl.y = self.control_heading
		self.pub.publish(ctrl)

	# 判断两条线段是否相交，false表示不相交，true表示相交
	# p1，p2是当前位置和目标位置，p3，p4是障碍物端点
	def cross(self, p1, p2, p3):
		x1 = p2[0] - p1[0]
		y1 = p2[1] - p1[1]
		x2 = p3[0] - p1[0]
		y2 = p3[1] - p1[1]
		return x1*y2 - x2*y1
	def intersec(self, p1, p2, p3, p4):
		if max(p1[0],p2[0])>=min(p3[0],p4[0]) and max(p3[0],p4[0])>=min(p1[0],p2[0]) and max(p1[1],p2[1])>=min(p3[1],p4[1]) and max(p3[1],p4[1])>= min(p1[1],p2[1]):
			if self.cross(p1,p2,p3)*self.cross(p1,p2,p4)<=0 and self.cross(p3,p4,p1)*self.cross(p3,p4,p2)<=0:
				return True
			else:
				return False
		else:
			return False
	# 用于计算点到线段的距离，p1表示点，l1、l2是线段的两个端点
	def caculateDistance(self, p1, l1, l2):
		k = (l1[1]-l2[1])/(l1[0]-l2[0])
		b = l1[1] - k*l1[0]
		length = abs(k*p1[0]-p1[1]+b)/((k*k+1)**0.5)
		return length

point = CtrlBoat()
point.listener()
		

