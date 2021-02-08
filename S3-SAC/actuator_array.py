from PyQt5.QtGui import QPainter, QPen, QBrush, QColor
from PyQt5.QtCore import QRect, QPoint, QSize, QTimer
import numpy as np
from math import pi
import random
import time
prob = np.array([0.92,2.7,3.6,3.6,14.41,36.04,9.01,7.21,2.7,4.5,9.01,3.6,1.8,0.36,0.54])/100
box = np.array([[40,30],
                [40,40],
                [60,40],
                [60,50],
                [80,60],
                [80,60],
                [120,80],
                [140,80],
                [82,56],
                [94,76],
                [220,200],
                [100,100],
                [168,188],
                [30,30],
                [200,140]])
box = box + 1      ## 变成奇数
# sample_index = np.random.multinomial(1, prob, size=1).argmax()

class Parcel():
    def __init__(self, p):
        # self.num = 1    # 默认数量为1
        # 刚体运动视为转动和平动的叠加
        # 宏观参数
        # 长宽高，先默认为长方体,视作刚体,最好长宽都是奇数
        # start = time.time()
        if 0:
            sample_index = np.random.multinomial(1, prob, size=1).argmax()
        else:       ### 训练时用等概率采样
            sample_index = np.random.randint(0,len(box))

        self.l = box[sample_index][0]
        self.w = box[sample_index][1]
        # self.l = 61
        # self.w = 41
        # end = time.time()
        # print("随机",end-start)

        # 运动时间
        self.T = 0

      # 步长为2,随机奇数
        self.h = 10     # 对理论分析没有意义

        self.k = 10   # 摩擦系数
        self.mu = 1000   # 柱密度函数，默认为常数
        self.m = self.mu*self.l*self.w

        self.J = 1/12*self.m*(self.l*self.l+self.w*self.w)  # 立方体的转动惯量
        self.theta = 0  # 当前旋转角，以逆时针方向为正
        # 力矩等于角加速度alpha乘转动惯量J

        # 微观参数
        # self.cal_Centroid()     # 质心所处的位置
        self.x = 0.0      # 质心所处的位置
        self.y = np.random.randint(0+(self.w-1)/2,250-(self.w-1)/2) + 0.0
        self.r_cm = np.array([self.x, self.y])

        self.rec = QRect(self.x-self.l/2, self.y - self.w/2, self.l, self.w)

        self.vx = 0.0     # 质心处初速度
        self.vy = 0.0
        # 质心处的速度,可能需要用向量表示；应该不需要，只有水平方向的速度；还是需要方便与后面的向量叠加,但是vy应该一直为0
        self.v_cm = np.array([self.vx, self.vy])

        # self.omega_x=0
        # self.omega_y=0
        self.omega = 0  # 10/180*pi    # 角速度 应该是一个向量,不他不是一个向量

        self.a_cm = [0, 0]  # 质心处加速度
        self.alpha = 0  # 质心处角加速度

        self.prio = 0
    
    def cal_Centroid(self):
        self.x = 0.0     
        self.y = np.random.randint(0 + (self.w-1)/2,201 - (self.w-1)/2) + 0.0
        self.r_cm = np.array([self.x, self.y])



class Actuator():
    def __init__(self):  # 1:5比例尺，单位mm
        self.h = 40
        self.w = 100
        self.color = QColor(0, 0, 255)
        self.speed = 0  # 600-1500/5 = 120-300mm/s
        self.state = 0
        self.speed_max = 60
        self.speed_min = 0


class Actuator_array(object):   # 包含了绘图属性和实际属性
    def __init__(self):
        super(Actuator_array, self).__init__()
        self.row = 6
        self.col = 5        ## 5-1
        self.gap_h = 2
        self.gap_w = 5
        self.start_x = 0
        self.start_y = 0

        self.rec = []       # 存储rec的list

        self.actuator = []
        for i in range(0, self.row*self.col+1):
            self.actuator.append(Actuator())

        # self.parcels = []                 # 存储包裹,数量可变所以不存在在执行器阵列中
        # self.parcels.append(Parcel())     #先存放一个包裹
        self.construct_array()
        # self.construct_parcel()

    def construct_array(self):
        rec1 = QRect(self.start_x, self.start_y,
                     self.actuator[0].w, self.row*self.actuator[0].h+(self.row-1)*self.gap_h)
        self.rec.append(rec1)

        size = QSize()
        size.setHeight(self.actuator[1].h)
        size.setWidth(self.actuator[1].w)

        for i in range(0, self.row):
            for j in range(0, self.col):
                index = i*self.col+j+1
                x = self.start_x + \
                    self.actuator[index].w+self.gap_w + \
                    self.actuator[index].w*j+self.gap_w*j
                y = self.start_y+self.actuator[index].h*i+self.gap_h*i
                temp = QPoint(x, y)
                self.rec.append(QRect(temp, size))

    # def construct_parcel(self):
    #     parcel = Parcel()
    #     self.parcels.append(parcel)     #先存放一个包裹


if __name__ == "__main__":
    act_array = Actuator_array()

    # act_array = Actuator_array()  # 全局变量
