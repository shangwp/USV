#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import time
import numpy as np
import rospy
import math
from matplotlib.patches import Ellipse,Circle
from geometry_msgs.msg import Vector3
from sys_sim.msg import globalObstacle
from sys_sim.msg import usv
#plt.switch_backend('agg')
boatCoordination = [0,0,0]
class Map():
    # 初始化输入 图片尺寸figx*figy，x轴最小值，x轴最大值，y轴最小值，y轴最大值
    def __init__(self, figX=20, figY=20, minX=0, maxX=500, minY=-130, maxY=130):
        plt.figure(1,figsize=(figX, figY))
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY

    # 用于在地图上画圆，输入参数为x轴坐标，y轴坐标，圆半径
    def drawCircle(self, a, b, r,c):
        theta = np.arange(0, 2*np.pi, 0.01)
        # x = a + r * np.cos(theta)
        # y = b + r * np.sin(theta)
        col='black'
        c=ord(c)
        #print(c)
        if c == 0:
            col = 'black'
        elif c == 1:
            col = 'red'
        elif c == 2:
            col = 'blue'
        elif c == 3:
            col = 'green'
        elif c == 4:
            col = 'orange'
        plt.scatter(a,b,color=col,s=20)
        # plt.plot(x,y)
    # 用于在地图上画多边形，输入为多边形的坐标点集，即[[x1,y1],[x2,y2],...]，注意坐标点需要按顺序绕多边形一周
    def drawPylon(self, point):
        x = []
        y = []
        for p in point:
            x.append(p[0])
            y.append(p[1])
        x.append(point[0][0])
        y.append(point[0][1])
        plt.plot(x, y)

    # 在地图上画船，船的图形已经定义，输入参数为x轴坐标，y轴坐标，船头朝向（度为单位，北为0，顺时针为正）
    def drawBoat(self, x, y, z):
        
        plt.arrow(x,y,2*math.sin(z),2*math.cos(z),head_width=1,length_includes_head=True)
        #plt.Circle((10,10),10,color='g')
        # z = -z*np.pi/180
        # point = [[x-0.25, y+0.5], [x, y+1], [x+0.25, y+0.5], [x+0.25, y-0.5], [x-0.25, y-0.5]]
        # listX = []
        # listY = []
        # for p in point:
        #     tmpX = x + (p[0]-x)*np.cos(z) - (p[1]-y)*np.sin(z)
        #     tmpY = y + (p[1]-y)*np.cos(z) + (p[0]-x)*np.sin(z)
        #     listX.append(tmpX)
        #     listY.append(tmpY)
        # listX.append(listX[0])
        # listY.append(listY[0])
        # plt.plot(listX, listY)

    # 绘制完所有图形后使用一次即可，用于补充图片细节
    def show(self):
        # 规定横纵坐标的范围
        plt.xlim((self.minX, self.maxX))
        plt.ylim((self.minY, self.maxY))
        # 规定横纵坐标每一格对应的距离，默认设置为10m
        my_x_ticks = np.arange(self.minX, self.maxX,10)
        my_y_ticks = np.arange(self.minY, self.maxY,10)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        # 图片标题
        plt.title("test")
        # 显示线条
        plt.grid(True)
        # 图片暂停时间，单位为秒
        plt.pause(0.01)
        plt.plot()
        # plt.get_backend()
    # 清空图片显示内容，建议在绘制图片前使用
    def clear(self):
        plt.clf()
        plt.cla()
        # plt.close()
    # 用于保存图片，尚未完善
    def save(self, name):
        #plt.savefig('./test2.jpg')
        plt.savefig(name)
def boatCallback(data):
    boatCoordination[0] = data.x
    boatCoordination[1] = data.y
    boatCoordination[2] = data.hdt
def callback(data):
    myMap = Map()
    # myMap.clear();
    plt.clf()
    plt.cla()
    print(data)
    for i in range(len(data.x)):
        myMap.drawCircle(data.x[i],data.y[i],data.radius[i],data.color[i])
    # for o in obstacleCoordination:
    #     myMap.drawCircle(o[0],o[1],o[2],o[3])
    myMap.drawBoat(boatCoordination[0],boatCoordination[1],boatCoordination[2])
    #myMap.drawCircle(boatCoordination[0],boatCoordination[1],50,2)
    myMap.show()

rospy.init_node('visualize', anonymous=True)
rospy.Subscriber("usv", usv,boatCallback)
rospy.Subscriber("globalObstacle",globalObstacle, callback)
rospy.spin()
