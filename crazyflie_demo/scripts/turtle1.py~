#! /usr/bin/env python 
# -*- coding: UTF-8 -*-

import roslib
import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import time
import random
import geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from datetime import datetime

# 全局参数
n = 5
f = 1
T = 0.1
alpha = 0.4 / (n * T * T)
beta = 3 / (2 * T)
m = 10000
h = 2.0
hx = [0.0, -0.5 * h / 3, 0.5 * h / 3, 0.0, 0.1 * h, -0.1 * h]
hy = [0.0, -0.2 * h / 3, -0.2 * h / 3, -0.5 * h / 3, 0.4 * h / 3, 0.4 * h / 3]
kv = [0.0, 1.0, 2.0, 1.5, 1.75, 1.25]
max_vel = 0.1
max_w = 0.5
x = np.zeros((n + 1, m), float)
y = np.zeros((n + 1, m), float)
w = np.zeros((n + 1, m), float)
vx = np.zeros((n + 1, m), float)
vy = np.zeros((n + 1, m), float)
vw = np.zeros((n + 1, m), float)
ux = np.zeros((n + 1, m), float)
uy = np.zeros((n + 1, m), float)
timeRecord = [] #记录时间计算速度
# 保存最佳历史值
xp = np.zeros((n + 1, m), float)
yp = np.zeros((n + 1, m), float)

# 参考速度输入
vrx = np.zeros((n + 1, m), float)
vry = np.zeros((n + 1, m), float)


def field2(x, y):
    return -(x * x + y * y)


# 将四元数转化成row, pitch, yaw
def quatToRPY(x, y, z, w):
    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * z))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    # angleR = r * 180 / math.pi
    # angleP = p * 180 / math.pi
    return y


# 去掉f个最大值和最小值
def fliter(temp, value):
    if max(temp) > value:
        temp.remove(max(temp))
    if min(temp) < value:
        temp.remove((min(temp)))


# 2范数
def norm(x, y):
    return math.sqrt(-field2(x, y))


# 限制速度最大值
def limitVel(v, wFlag=False):
    if wFlag:
        maxValue = max_w
    else:
        maxValue = max_vel
    if v > maxValue:
        return maxValue
    elif v < -maxValue:
        return -maxValue
    else:
        return v


class resilient():
    def __init__(self):
        self.xCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 保存每次在回调函数中更新的最新数据，当达到迭代时间后进行更新
        self.yCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.wCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vxCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vyCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vwCurrent = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.count = 0

        self.sub1 = rospy.Subscriber("/vrpn_client_node/r1/pose", PoseStamped, self.callback1)
        self.sub2 = rospy.Subscriber("/vrpn_client_node/r2/pose", PoseStamped, self.callback2)
        self.sub3 = rospy.Subscriber("/vrpn_client_node/r3/pose", PoseStamped, self.callback3)
        self.sub4 = rospy.Subscriber("/vrpn_client_node/r4/pose", PoseStamped, self.callback4)
        self.sub5 = rospy.Subscriber("/vrpn_client_node/r5/pose", PoseStamped, self.callback5)

    def delta(self):
        k = self.count
        return 1.0 / (math.pow(k, 0.2) + 1.0)

    def update(self):
        print("update process")
        k = random.random()
        d = math.sin(k * math.pi * 2) + 1  # 模拟攻击
        num = self.count
        if num < 1:
            for i in range(1, 6):
                x[i][num] = self.xCurrent[i]
                y[i][num] = self.yCurrent[i]
                w[i][num] = self.wCurrent[i]
                xp[i][num] = self.xCurrent[i]
                yp[i][num] = self.yCurrent[i]
                vx[i][num] = max_vel
                vy[i][num] = max_vel
                vw[i][num] = max_vel
                vrx[i][num] = max_vel
                vry[i][num] = max_vel
            self.count += 1
        else:
            for i in range(1, 6):
                x[i][num] = self.xCurrent[i]
                y[i][num] = self.yCurrent[i]
                w[i][num] = self.wCurrent[i]
                if field2(x[i][num], y[i][num]) >= field2(xp[i][num - 1], yp[i][num - 1]):
                    xp[i][num] = x[i][num]
                    yp[i][num] = y[i][num]
                else:
                    xp[i][num] = xp[i][num - 1]
                    yp[i][num] = yp[i][num - 1]
            self.count += 1

        if num % 10 == 9:  ##record
            np.savetxt("x.txt", x[1:, :num])
            np.savetxt("y.txt", y[1:, :num])
            np.savetxt("w.txt", w[1:, :num])
            np.savetxt("vx.txt", vx[1:, :num])
            np.savetxt("vy.txt", vy[1:, :num])
            np.savetxt("vw.txt", vw[1:, :num])
            np.savetxt("xp.txt", xp[1:, :num])
            np.savetxt("yp.txt", yp[1:, :num])
        # print("num: ", num)
        for i in range(1, 6):
            a = random.randint(1, 5)  # 随机选取一个机器人受到攻击
            tempx = []  # 临时储存位置信息
            tempy = []
            for j in range(1, 6):
                if j == i:
                    continue
                elif j == a:
                    tempx.append(d)
                    tempy.append(d)
                else:
                    tempx.append(x[j][num])
                    tempy.append(y[j][num])

            print('temp_before', tempx, tempy)
            fliter(tempx, x[i][num])
            fliter(tempy, y[i][num])

            if num > 1:
                if xp[i][num] == x[i][num]:
                    vrx[i][num] = 2 * vx[i][num - 1] / norm(vx[i][num - 1], vy[i][num - 1])
                    vry[i][num] = 2 * vy[i][num - 1] / norm(vx[i][num - 1], vy[i][num - 1])
                else:
                    vrx[i][num] = (xp[i][num] - x[i][num]) / norm(xp[i][num] - x[i][num], yp[i][num] - y[i][num])
                    vry[i][num] = (yp[i][num] - y[i][num]) / norm(xp[i][num] - x[i][num], yp[i][num] - y[i][num])

            ux[i][num] =  alpha * (sum(tempx) + len(tempx) * hx[i]) -  alpha * len(tempx) * \
                         x[i][num] - beta * (vx[i][num] - kv[i] * self.delta() * vrx[i][num])

            uy[i][num] =  alpha * (sum(tempy) + len(tempy) * hy[i]) -  alpha * len(tempy) * \
                         y[i][num] - beta * (vy[i][num] - kv[i] * self.delta() * vry[i][num])


            print('temp', tempx, tempy)
            print('ux', i, self.delta() * alpha * (sum(tempx) + len(tempx) * hx[i]),
                  self.delta() * alpha * len(tempx) * x[i][num], beta * (vx[i][num] - 2.5 * self.delta() * \
                                                                         vrx[i][num]))
            print('uy', i, self.delta() * alpha * (sum(tempy) + len(tempy) * hy[i]),
                  self.delta() * alpha * len(tempy) * y[i][num], beta * (vy[i][num] - 2.5 * self.delta() * \
                                                                         vry[i][num]))
        self.publish(ux, uy, num)

    def publish(self, ux, uy, num):
        timeRecord.append(datetime.now())
        print("publish process")
        robot = ['/r0', '/r1', '/r2', '/r3', '/r4', '/r5']
        for i in range(1, 6):
            pub = rospy.Publisher(robot[i] + '/cmd_vel', Twist, queue_size=10)
            msg = Twist()
            if num == 0:
                linearx = limitVel(ux[i][num] * T)
                wz = 0.0
            else:
            	#print("t:", datetime.now(), (timeRecord[num] - timeRecord[num - 1]).microseconds)
                vx[i][num - 1] = 1000 * (x[i][num] - x[i][num - 1]) / (timeRecord[num] - timeRecord[num - 1]).microseconds
                vy[i][num - 1] = 1000 * (y[i][num] - y[i][num - 1]) / (timeRecord[num] - timeRecord[num - 1]).microseconds
                vw[i][num - 1] = 1000 * (w[i][num] - w[i][num - 1]) / (timeRecord[num] - timeRecord[num - 1]).microseconds
                linearx = vx[i][num - 1] + ux[i][num] * T
                lineary = vy[i][num - 1] + uy[i][num] * T
                theta = math.atan2(lineary, linearx)
                '''if linearx < 0:  # 车头在x轴，因此1,4象限不变，2,3差一个pi
                    if theta > 0:
                        theta -= math.pi
                    else:
                        theta += math.pi'''
                angular = theta - w[i][num]
                # 总是使机器人朝小角度旋转
                if angular < 0:
                    if abs(angular) > abs(angular + 2 * math.pi):
                        angular += 2 * math.pi
                elif angular > 0:
                    if abs(angular) > abs(angular - 2 * math.pi):
                        angular -= 2 * math.pi
                
                if abs(angular) > math.pi / 4:
                    wz = limitVel(1.0 * (vw[i][num - 1] + angular * T), True)
                    linearx =  norm(linearx, lineary)
                    print("vel & angular", linearx, angular)
                    if linearx > 0.1:
                        linearx = 0.02 * linearx

                else:
                    wz = limitVel(1.0 * (vw[i][num - 1] + angular * T), True)
                    linearx = norm(linearx, lineary)
                if abs(wz) < 0.05:
                    wz = 0
                if abs(linearx) < 0.01:
                    linearx = 0.0
            msg.angular.z = wz
            msg.linear.x = limitVel(linearx)
            print(msg.linear.x, msg.angular.z)
            pub.publish(msg)

    def callback1(self, data):
        try:
            self.xCurrent[1] = data.pose.position.x
            self.yCurrent[1] = data.pose.position.y
            quat = data.pose.orientation
            self.wCurrent[1] = quatToRPY(quat.x, quat.y, quat.z, quat.w)
        except rospy.ROSInterruptException:
            pass

    def callback2(self, data):
        try:
            self.xCurrent[2] = data.pose.position.x
            self.yCurrent[2] = data.pose.position.y
            quat = data.pose.orientation
            self.wCurrent[2] = quatToRPY(quat.x, quat.y, quat.z, quat.w)
        except rospy.ROSInterruptException:
            pass

    def callback3(self, data):
        try:
            self.xCurrent[3] = data.pose.position.x
            self.yCurrent[3] = data.pose.position.y
            quat = data.pose.orientation
            self.wCurrent[3] = quatToRPY(quat.x, quat.y, quat.z, quat.w)
        except rospy.ROSInterruptException:
            pass

    def callback4(self, data):
        try:
            self.xCurrent[4] = data.pose.position.x
            self.yCurrent[4] = data.pose.position.y
            quat = data.pose.orientation
            self.wCurrent[4] = quatToRPY(quat.x, quat.y, quat.z, quat.w)
        except rospy.ROSInterruptException:
            pass

    def callback5(self, data):
        try:
            self.xCurrent[5] = data.pose.position.x
            self.yCurrent[5] = data.pose.position.y
            quat = data.pose.orientation
            self.wCurrent[5] = quatToRPY(quat.x, quat.y, quat.z, quat.w)
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    rospy.init_node('resilient')
    res = resilient()
    while not rospy.is_shutdown():
        try:
            res.update()
            # rospy.spin()
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            print("shut down")



