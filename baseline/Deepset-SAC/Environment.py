from simulation import Physic_simulation
from simulation import act
import numpy as np
import actuator_array
import time
import copy
import actuator_array as act
from init import action_dim,state_dim,col_num,Parcel_number,state_per_dim
 



Additional_parcel = 6       ## 输入进网络的只有前6个包裹，实际上有11个包裹，提高网络的泛化性


class Environment(Physic_simulation):
    def __init__(self):
        self.finish_tmp = 0
        self.T = 0
        self.cal_dist = 0
        self._build()


    def _build(self):
        # self.finish_time = []
        self.finish_flag = 0        # 声明继承Physic_simulation的属性
        self.evl_flag = 0           # evl时统计时间差
        self.delta_t = 0
        self.Parcels = []
        self.act_array = act.Actuator_array() 
        self.Generate_parcels()

    def reset(self):
        # self.finish_time = []
        self.finish_flag = 0
        self.delta_t = 0
        self.Parcels = []
        self.act_array = act.Actuator_array() 
        self.Generate_parcels()
        # 返回两个包裹的位置信息  加x方向速度
        loc = []
        i = 0
        for parcel in self.Parcels:         ## 数据预处理，去均值，归一化
            if i < Parcel_number:
                # loc.extend([(parcel.x)/600])
                # loc.extend([(parcel.y)/250])
                # loc.extend([(parcel.l)/220])
                # loc.extend([(parcel.w)/200])
                loc.extend([(parcel.x - parcel.l/2)/600])
                loc.extend([(parcel.x + parcel.l/2)/600])
                loc.extend([(parcel.y - parcel.w/2)/250])
                loc.extend([(parcel.y + parcel.w/2)/250])
                loc.extend([(parcel.v_cm[0])/60])
                i = i + 1
        loc = np.array(loc)
        # loc = np.zeros(Parcel_number*state_per_dim)
        return loc   #dim=1

    def Sort_parcel(self):
        unsort_prio = []
        num = 0
        for parcel in self.Parcels:
            unsort_prio.append(parcel.x + parcel.l/2)
            if parcel.x + parcel.l/2 > 0:
                num = num + 1

        for index in range(0, len(self.Parcels)):            # 每个包裹的最大x值在当前包裹中的顺序作为优先级，可以近似为右边界比较
            self.Parcels[index].prio = np.where(np.sort(-np.array(unsort_prio))==-unsort_prio[index])[0][0]
        sort_index = np.argsort(np.array(unsort_prio))  # 升序下标

        return sort_index[-num:]    ## 取后num个数，从而保证输入网络的都是传送带上的

    def Padding_parcel(self,l):     ## 从生成包裹分布中重新采样
        loc = []
        count = 0
        h = 250
        while count < l:
            y_low = 0
            delta = np.random.randint(1,3)
            while y_low < h:
                temp1 = actuator_array.Parcel(2)
                temp1.x = 0.0 ##- (temp1.l - 1)/2 + 0.0
                temp1.y = delta + (temp1.w - 1)/2 + 0.0 + y_low
                y_low = y_low + temp1.w + delta
                temp1.r_cm = np.array([temp1.x, temp1.y])
                if count < l:
                    if y_low < h:
                        count = count + 1
                        loc.extend([(temp1.x - temp1.l/2)/600])
                        loc.extend([(temp1.x + temp1.l/2)/600])
                        loc.extend([(temp1.y - temp1.w/2)/250])
                        loc.extend([(temp1.y + temp1.w/2)/250])
                        loc.extend([(temp1.v_cm[0])/60])
                else:
                    break
        return loc

    def step(self, action):
        # take action


        # RL 修正
        for i in range(0,action_dim):
            j = i
            if i > (col_num - 1):
                j = i+1
            if i > (col_num - 1)*2:
                j = i+2
            if i > (col_num - 1)*3:
                j = i+3
            if i > (col_num - 1)*4:
                j = i+4     
            if i > (col_num - 1)*5:
                j = i+5 
            if i > (col_num - 1)*6:
                j = i+6       
            self.act_array.actuator[j].speed = action[i]
            if self.act_array.actuator[j].speed > self.act_array.actuator[0].speed_max:      ## 限幅
                self.act_array.actuator[j].speed = self.act_array.actuator[0].speed_max
            if self.act_array.actuator[j].speed < self.act_array.actuator[0].speed_min:
                self.act_array.actuator[j].speed = self.act_array.actuator[0].speed_min


        ## 测试区域不可控
        self.act_array.actuator[5].speed = 60       ## 训练完后降低速度，效率大大提高
        self.act_array.actuator[10].speed = 60
        self.act_array.actuator[15].speed = 60
        self.act_array.actuator[20].speed = 60
        self.act_array.actuator[25].speed = 60
        self.act_array.actuator[30].speed = 60

        # next step
        self.Parcel_sim()

        r_normal = -0.01*len(self.Parcels)           ### 放在仿真完成之后计算，否则计算出的是常数


        while len(self.Parcels) < np.random.randint(12,13):        ## 仿真环境中包裹数量随机
            self.Add_parcels()
        # s_

        loc = []

        sort_index = self.Sort_parcel()

        # print("排序",start2-start1)

        # 输入包裹信息为前N个包裹，并且要按照顺序排序？，不按照顺序排序没有泛化性
        for i,index in enumerate(sort_index[::-1]):     ## 逆序 ## 正常应该输入大于0的包裹到网络中才能保持稳定性
            if i < Parcel_number:       # i 从0开始，最大不超过预设 Parcel_number
                parcel = self.Parcels[index]

                loc.extend([(parcel.x - parcel.l/2)/600])
                loc.extend([(parcel.x + parcel.l/2)/600])
                loc.extend([(parcel.y - parcel.w/2)/250])
                loc.extend([(parcel.y + parcel.w/2)/250])
                loc.extend([(parcel.v_cm[0])/60])
                

        l = Parcel_number - len(sort_index)                                    # 只有在传送带上的才会输入进网络
        if l <= 0:       ## 传送带上包裹超过预设的大小
            l = 0

        s_ = np.array(loc)

        s_ = np.pad(s_,[0,(l)*state_per_dim])            # 自动补0

        # r_normal = 0        # 可能不需要
        r_finish = 0


        done = 0

        sum_v = 0
        if len(self.Parcels)!=0:
            for parcel in self.Parcels:
                sum_v = sum_v + parcel.v_cm[0]
            if sum_v < 5:      ## 都没有在动，说明卡死，重启            不能以0判断，有可能都小于1然后就卡死
                done = 1
                r = -20
                # print("done")

                return s_,r,done

            # if int(self.Parcels[0].v_cm[0]) == 0 and int(self.Parcels[1].v_cm[0]) == 0:
            #     done = 1
            #     r = -20
            #     return s_,r,done

        if self.finish_flag == 1:       ## 有包裹过线
            self.finish_flag = 0
            self.evl_flag = 1

                # x1 = [0,150-150*0.1,150-150*0.05,150];
                # x2 = [150,450];
                # x = [x1,x2];
                # y1 = [-20,-10,0,10];
                # y2 = [10,0];
                # y = [y1,y2];
                
            if self.cal_dist >= 150:##and delta_t <= 6:
                r_finish = -0.03333*self.cal_dist + 15        ## 150 - 450 都有奖励 


            else:
                # r_finish = -20
                r_finish = 20*((self.cal_dist-150)/150)        ## 150_0  0_-20
                # r_finish = np.power(1.071,(self.cal_dist - 100))- 20.87
            # if delta_t >6:
            #     r_finish = -10
            # print("两个包裹相差时间\t",round(delta_t,1),"\tr_finish\t", round(r_finish,1))
            # print(delta_t)
        r = r_normal + r_finish
        return s_,r,done