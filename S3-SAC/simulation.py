###### 仿真建模文件
## 包含像素级分析的包裹生成，每个包裹计算合力，合力矩，分析下一时间的受力情况
from math import pi, cos, sin, floor
import sys
# import matplotlib.pyplot as plt
import numpy as np
import actuator_array
import time
import copy
import actuator_array as act

from init import action_dim,state_dim,Parcel_number
import my_swig.example as C
Pixel_array = []
T = 0
finish_tmp = 0
act_temp = act.Actuator_array()
w = act_temp.actuator[0].w
h = act_temp.row * \
        act_temp.actuator[0].h+(act_temp.row-1)*act_temp.gap_h

dest = w + (act_temp.col)*act_temp.actuator[1].w + (act_temp.col)*act_temp.gap_w     ## -1扣除最后一列的测试传送带区域，速度不可控

add_test_area = 100 ## 测试区域长度为最后一列+add_test_area = 100 + 100 = 200


finish_num = 0
finish_5 = 0
finish_5_10 = 0
finish_10_20 = 0
finish_20 = 0

fail_num = 0
fail_5 = 0
fail_5_10 = 0
fail_10 = 0

class Physic_simulation():

    def __init__(self): 
        self.Parcels = []           # 生成包裹需要考虑当前的像素占用情况
        self.act_array = act.Actuator_array() 
        self.finish_time = []
        self.finish_tmp = 0
        self.T = 0
        self.cal_dist = 0
    # 根据当前包裹位置生成仿真所需要的像素点   ?根据上一个时刻的包裹像素点的分布计算当前像素点的位置
    def Generate_pixel(self, subsample, index):

        parcel = self.Parcels[index]
        theta = parcel.theta/180*pi
        # M = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        pixel = np.ones(
            [(parcel.l+1)//subsample, (parcel.w+1)//subsample, 2])   # 生成包裹矩阵

        for i in range(0, (parcel.l+1)//subsample):
            for j in range(0, (parcel.w+1)//subsample):
                pixel[i][j] = ([parcel.x-(parcel.l-1)/2+i*subsample,
                                parcel.y - (parcel.w-1)/2+j*subsample])

        # pixel.resize((parcel.w+1)//subsample*(parcel.l+1) //
        #              subsample, 2)       # 拉成一维
        pixel = np.resize(pixel,((parcel.w+1)//subsample*(parcel.l+1) //
                subsample, 2))      # 拉成一维
        # pixel = np.dot(pixel-parcel.r_cm, M)+parcel.r_cm       # 旋转后的点
        return pixel

    # 用矩阵计算代替循环优化计算速度,经过分析，np.array占用大量时间，大致为0.14-0.18不可以在循环内进行转换
    def cal_Force(self, pixel, num_act, parcel):
        # 分与不同的传送带上的接触面积去计算受力情况
        F = np.array([0, 0])
        F_test = []
        for i in range(0, num_act.size):
            if num_act[i] == -1:
                speed = 0
            else:
                speed = self.act_array.actuator[int(num_act[i])].speed
            speed = int(speed)
            F = F + self.cal_force(speed, pixel[i], parcel.r_cm,
                                   parcel.v_cm, parcel.omega, parcel.mu, parcel.k, 1)
            # F_test.append(self.cal_force(self.act_array.actuator[int(num_act[i])].speed,pixel[i],1))
        F[1] = 0
        return F

    # 力矩产生平面上的旋转
    def cal_Torque(self, pixel, num_act, index):
        torque = 0
        parcel = self.Parcels[index]
        R = []
        f = []
        for i in range(0, num_act.size):
            R.append(pixel[i]-parcel.r_cm)
            if num_act[i] == -1:
                speed = 0
            else:
                speed = self.act_array.actuator[int(num_act[i])].speed
            f.append(self.cal_force(
                speed, pixel[i], parcel.r_cm, parcel.v_cm, parcel.omega, parcel.mu, parcel.k, 2))
        # 首先要保证受力后的力矩为0，主要考虑边界上，可能会误分,piexel无法做到完全的对称。。。艹烦躁
        # 减小因为pixel带来的转动惯量的影响，1增大pixel数量，2减小转动惯量的大小，这样较小的转动惯量对于整体的影响很小
        # 加速度和角加速度均与物体质量无关，所以还是需要增大接触面积
        torque = np.cross(R, f)          # 一次大量计算比多次少量计算快很多
        torque = sum(torque)
        return torque

    # 执行器速度，质点位置，质心位置，质心速度，角速度，mu,k
    def cal_force(self, act_speed, r, r_cm, V_cm, omega, mu, k, flag):

        ## C 底层
        f = C.cal_force(act_speed,r[0],r[1],r_cm[0],r_cm[1],V_cm[0],V_cm[1],omega,mu,k,flag)
        return [f,0]
        # # r_cm = parcel.r_cm
        # U_r = [act_speed, 0]      # 质点处传送带的速度       120-300mm/s
        # # V_cm = parcel.v_cm
        # # omega = parcel.omega
        # rotate = [[0, 1], [-1, 0]]

        # R = np.dot([r[0]-r_cm[0], r[1]-r_cm[1]], rotate)
        # Dv = 20                            # 振荡上升表面分界值太大或者力力矩太大,正负振荡表明分界值太小
        # Rv = 5
        # fr = 0
        # if flag == 1:       # 计算合力      ,假设速度大于Dv时与速度无关，小于Dv时与速度线性，Dv可以修正
        #     if U_r[0] - V_cm[0] > -Dv and U_r[0] - V_cm[0] < Dv:      # 相对速度差比较
        #         fr = mu*k*(U_r - V_cm)/Dv
        #     else:
        #         fr = mu*k*(U_r - V_cm)/np.linalg.norm(abs((U_r - V_cm)))

        # else:               # 计算合力矩，质心处速度的合力矩为0，所以只考虑后半部分,omega逆时针为正所以速度方向与U_R相反
        #     if np.linalg.norm(U_r - V_cm + omega*R) > -Rv and np.linalg.norm(U_r - V_cm + omega*R) < Rv:
        #         fr = mu*k*(U_r - V_cm + omega*R)/Rv
        #     else:
        #         fr = mu*k*(U_r - V_cm + omega*R) / \
        #             np.linalg.norm(U_r - V_cm + omega*R)
        # return fr

    # 判断质点r处于哪个执行器上
    def cal_num_act(self, pixel):
        # 超出边界的像素不参与计算
        bound = self.act_array.actuator[1].w*(self.act_array.col+1)+self.act_array.gap_w*(self.act_array.col) 
        for pix in pixel:
            if pix[0]>bound:
                pix[0] = bound
            if pix[0]<0:
                pix[0] = 0
        # pixel[pixel[:,0]>bound] = [bound,pixel[:,1]]
        # pixel[pixel[:,0]<0] = 0

        # 计算num_act矩阵
        quotient_x = pixel[:,
                           0]//(self.act_array.actuator[1].w+self.act_array.gap_w)  # 第几列
        remainder_x = pixel[:, 0] % (self.act_array.actuator[1].w+self.act_array.gap_w)

        # quotient_x[ quotient_x> self.act_array.actuator[1].w] = self.act_array.actuator[1].w      # 限制列

        quotient_y = pixel[:, 1]//(self.act_array.actuator[1].h +
                                   self.act_array.gap_h)    # 第几行
        remainder_y = pixel[:, 1] % (self.act_array.actuator[1].h+self.act_array.gap_h)

        # quotient_y[ quotient_y> self.act_array.actuator[1].h] = self.act_array.actuator[1].h      # 限制行

        num_act = quotient_y*self.act_array.col + quotient_x

        for i in range(0, num_act.size):
            if remainder_x[i] > self.act_array.actuator[1].w:  # 落在gap中，可以直接返回
                num_act[i] = -1
            if quotient_x[i] != 0 and remainder_y[i] > self.act_array.actuator[1].h:  # 落在gap中，直接返回
                num_act[i] = -1
            if quotient_x[i] == 0:
                num_act[i] = 0
            if num_act[i] < -1:  # 定上下限
                num_act[i] = 0
            elif num_act[i] > self.act_array.col*self.act_array.row:  # 0-32  超过上限不受力
                num_act[i] = -1

        return num_act

    def Parcel_sim(self):
 
        start1 = time.time()
        self.Parcel_analysis()
        start2 = time.time()

        self.update_parcels()

        # print("包裹分析",start2-start1)
        # self.update_pixel()

    def Parcel_analysis(self):  # 计算每个包裹受力，进行位移更新
        global T,finish_tmp
        # temp_time = 0
        T = T + 1
        self.T = T
 
        # 计算每个包裹的受力情况，后面可以改成矩阵运算加快计算速度
        for index in range(0, len(self.Parcels)):         # 存在先后问题，不能直接删除
            parcel = self.Parcels[index]

            sub = 8
            # subsample 注意原来的长宽高+1/subsample 需要为整数

            pixel = self.Generate_pixel(sub, index)    # 放在类里
            num_act = self.cal_num_act(pixel)

            # #更新parcel中心点的位置
            # start = time.time()
            F = self.cal_Force(pixel, num_act, parcel)*sub*sub       # 质量 受力

            # end = time.time()
            # print(F)
            parcel.a_cm = (F)/(parcel.m)
            # print(parcel.a_cm[0]/parcel.v_cm[0])   ### 大于-1,小于-1会产生振荡
            parcel_v_cm = parcel.v_cm + 1*parcel.a_cm

            # 检测碰撞，如果碰撞，则速度为0
            overlap_flag, overlap_index = self.overlap_detect(index,parcel_v_cm[0])
            if overlap_flag:
                parcel_v_cm = self.Parcels[overlap_index].v_cm  # 速度相同
                self.Parcels[overlap_index].v_cm = self.Parcels[overlap_index].v_cm #+ np.array([5,0])   # 被碰撞包裹速度增加
                parcel_x = self.Parcels[overlap_index].x - (self.Parcels[overlap_index].l - 1)/2 - (parcel.l - 1)/2 - 2        # 距离相切
                pass
            else:
                try:
                    # 一个单位时间更新一次，减小刷新步长
                    parcel_x = parcel.x + np.floor(parcel_v_cm[0]*1)       
                except:
                    print(1)

            # 在计算完旋转角之后更新位移

            parcel.v_cm = parcel_v_cm
            parcel.x = parcel_x
            parcel.r_cm[0] = parcel_x

    def overlap_detect(self,index,parcel_v_cm):        # index指当前的包裹下标
        P = self.Parcels[index]
        overlap_flag = 0
        for i in range(0, len(self.Parcels)):          # 先找距离最近的并且可能超过他的
            if i != index:          # 与不是自己比较
                target = self.Parcels[i]     # 比较对象
                tmp_x = 1000
                # if P.x<target.x and abs(target.x-P.x) <=((target.l - 1)/2+(P.l - 1)/2) + parcel_v_cm + 2 and abs(target.y-P.y)<=((target.w - 1)/2+(P.w - 1)/2): #可能会发生碰撞
                if P.x + (P.l-1)/2<target.x + (target.l-1)/2 and abs(target.x-P.x) <=((target.l - 1)/2+(P.l - 1)/2) + parcel_v_cm + 2 and abs(target.y-P.y)<=((target.w - 1)/2+(P.w - 1)/2): #可能会发生碰撞
                        overlap_flag = 1
                        if target.x < tmp_x:        # 找出最近的碰撞包裹下标
                            tmp_index = i
                            tmp_x = target.x
        if overlap_flag:
            return 1,tmp_index
        else:
            return 0,0


    def update_parcels(self):       ## 更新self.parcels中的包裹，如果越界，则删除
        parcels_temp = []

        for index in range(0, len(self.Parcels)):         # 更新完所有包裹的位置和速度后判断是否出界
            parcel = self.Parcels[index]

            # 判断是否越过测试界限，如果超过 将Parcel删除    dest之后的区域的执行器速度恒定
            if((parcel.x - (parcel.l-1)/2) < dest + add_test_area):     ### 仍然保留越过dest的包裹，用于测试
                parcels_temp.append(parcel)

            if((parcel.x - (parcel.l-1)/2) >= dest + add_test_area):      ### 包裹的左半边越界，计算该包裹的右边界和测试区域包裹左边界的距离
                parcel.T = T + (parcel.x - (parcel.l-1)/2 - (dest + add_test_area))/parcel.v_cm[0]     ## 实测时间
                ## 实际左边界到达dest时间差
                offset_t = (parcel.x - (parcel.l-1)/2 - (dest + add_test_area))/parcel.v_cm[0]

                ## 计算第一次左边界过线包裹左边界和上一个包裹的右边界距离差     ## 需要每次都重新排序？

                ## 遍历所有包裹，选出距离最近的
                tmp_dist = 1000
                tmp_index = 0
                for j in range(0, len(self.Parcels)):
                    if j != index:
                        target = self.Parcels[j]
                        right = target.x + ((target.l-1)/2)
                        if parcel.x - (parcel.l-1)/2 - right < tmp_dist:
                            tmp_dist = parcel.x - (parcel.l-1)/2 - right
                            tmp_index = j
                delta_x = tmp_dist

                ## 校准距离差 为 两个相邻包裹的速度（上一个速度）*实际到达dest的时间差
                cal_dist = delta_x - (parcel.v_cm[0] - self.Parcels[tmp_index].v_cm[0]) * offset_t
                self.cal_dist = cal_dist

                global finish_num,fail_num,fail_5,fail_5_10,fail_10,finish_5,finish_5_10,finish_10_20,finish_20
                finish_num = finish_num + 1

                per = 5
                if cal_dist < 150:      ## 不合格
                    fail_num = fail_num + 1
                if cal_dist < 150 and cal_dist >= 150 - 0.05*150:
                    fail_5  = fail_5 + 1
                if cal_dist < 150 - 0.05*150 and cal_dist >= 150 - 0.1*150:
                    fail_5_10 = fail_5_10 + 1
                if cal_dist < 150 - 0.1*150:
                    fail_10 = fail_10 + 1

                if cal_dist > 150 and cal_dist <= 150 + 0.05*150:
                    finish_5 = finish_5 + 1
                if cal_dist > 150 + 0.05*150 and cal_dist <= 150 + 0.1*150:
                    finish_5_10 = finish_5_10 + 1
                if cal_dist > 150 + 0.1*150 and cal_dist <= 150 + 0.2*150:
                    finish_10_20 = finish_10_20 + 1
                if cal_dist > 150 + 0.2*150:
                    finish_20 = finish_20 + 1
                    
                    

                print("两个包裹距离间隔",round(cal_dist,1),round(delta_x,1),"过线数量",finish_num
                ,"不合格数量",fail_num,"小于5%",fail_5,"小于5%~10%",fail_5_10,"小于10%",fail_10
                ,"大于5%内",finish_5,"大于5%~10%",finish_5_10,"大于10~20%",finish_10_20,"大于20%",finish_20)

                if self.finish_tmp!=0:
                    self.finish_flag = 1        # 包裹过线标志位
                    self.delta_t = parcel.T - self.finish_tmp
                    if 0:
                        print(round(self.delta_t,1))
                self.finish_tmp = parcel.T

        # print("仿真受力计算时间",str(temp_time))
        self.Parcels = parcels_temp      # 删除越界的包裹

    def update_pixel(self):
        # 更新pixel
        parcels_pixel = []

        for index in range(0, len(self.Parcels)):         # 存在先后问题，不能直接删除
            try:
                parcel = self.Parcels[index]
            except:
                print()
            sub = 1         # 生成包裹所需要的像素信息不能降采样
            pixel = self.Generate_pixel(sub, index)
            parcels_pixel.append(pixel)

        global w,h
        # 生成执行器1上的像素点矩阵


        act1_matrix = np.zeros([w+2, h+2])

        for parcel_pix in parcels_pixel:
            for pix in parcel_pix:
                x = pix[0]
                y = pix[1]
                if x>=0:
                    if x <= w and y<=h:  #必须取等号
                        act1_matrix[int(x), int(y)] = 1


    ## 只需要self.Parcels和act_array。
    ## self.Parcels里应该记录当前所有包裹占用的像素点        
    ## add_parcels，生成起始线后面的一排包裹。
    def Add_parcels(self):      # y 方向生成，x方向靠碰撞拉伸
        y_low = 0
        delta = np.random.randint(1,3)
        global h
        while y_low < h:
            temp1 = actuator_array.Parcel(2)
            temp1.x = 0 - (temp1.l - 1)/2 + 0.0
            temp1.y = delta + (temp1.w - 1)/2 + 0.0 + y_low
            y_low = y_low + temp1.w + delta
            temp1.r_cm = np.array([temp1.x, temp1.y])
            if y_low < h:
                self.Parcels.append(copy.deepcopy(temp1))


    def Generate_parcels(self):
        while len(self.Parcels) < np.random.randint(10,12):
            self.Add_parcels()
            # for parcel in self.Parcels:
            #     parcel.x = parcel.x + 5
            #     parcel.r_cm[0] = parcel.r_cm[0] + 5
        # num = int(Parcel_number/3)
        # for i in range(num):                        ## 前两行
        #     temp1 = actuator_array.Parcel(2)
        #     temp1.x = 0+i*20.0        ## 避免生成时产生重叠
        #     temp1.y = 40.0
        #     temp1.r_cm = np.array([temp1.x, temp1.y])
        #     self.Parcels.append(copy.deepcopy(temp1))
        # for i in range(num):
        #     temp1 = actuator_array.Parcel(2)
        #     temp1.x = 0+i*20.0        ## 避免生成时产生重叠
        #     temp1.y = 100.0
        #     temp1.r_cm = np.array([temp1.x, temp1.y])
        #     self.Parcels.append(copy.deepcopy(temp1))
        # for i in range(num):
        #     temp1 = actuator_array.Parcel(2)
        #     temp1.x = 0+i*20.0        ## 避免生成时产生重叠
        #     temp1.y = 160.0
        #     temp1.r_cm = np.array([temp1.x, temp1.y])
        #     self.Parcels.append(copy.deepcopy(temp1))


    def isoccupy(self, pixel):  # 判断像素点是否在当前像素矩阵中被占用
        # start = time.time()
        # for i in range(0,10000):
        try:
            for parcel in self.Parcels:          
                if pixel in parcel.pixel:
                    return 1
                # if act1_matrix[pixel[0]][pixel[1]] == 1:
                #     return 1
        except:
            return 1  ##溢出直接不满足插入条件

        return 0


