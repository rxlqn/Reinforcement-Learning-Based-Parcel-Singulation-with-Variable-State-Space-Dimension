import example
import time
import numpy as np
print('My_varaiable: %s' % example.cvar.My_variable)
print('fact(5): %s' % example.fact(5))
print('my_mod(7,3): %s' % example.my_mod(7,3))
print('get_time(): %s' % example.get_time())

def cal_force(act_speed, r, r_cm, V_cm, omega, mu, k, flag):

    # r_cm = parcel.r_cm
    U_r = [act_speed, 0]      # 质点处传送带的速度       120-300mm/s
    # V_cm = parcel.v_cm
    # omega = parcel.omega
    rotate = [[0, 1], [-1, 0]]

    R = np.dot([r[0]-r_cm[0], r[1]-r_cm[1]], rotate)
    Dv = 20                            # 振荡上升表面分界值太大或者力力矩太大,正负振荡表明分界值太小
    Rv = 5
    fr = 0
    if flag == 1:       # 计算合力      ,假设速度大于Dv时与速度无关，小于Dv时与速度线性，Dv可以修正
        if U_r[0] - V_cm[0] > -Dv and U_r[0] - V_cm[0] < Dv:      # 相对速度差比较
            fr = mu*k*(U_r - V_cm)/Dv
        else:
            fr = mu*k*(U_r - V_cm)/np.linalg.norm(abs((U_r - V_cm)))

    else:               # 计算合力矩，质心处速度的合力矩为0，所以只考虑后半部分,omega逆时针为正所以速度方向与U_R相反
        if np.linalg.norm(U_r - V_cm + omega*R) > -Rv and np.linalg.norm(U_r - V_cm + omega*R) < Rv:
            fr = mu*k*(U_r - V_cm + omega*R)/Rv
        else:
            fr = mu*k*(U_r - V_cm + omega*R) / \
                np.linalg.norm(U_r - V_cm + omega*R)
    return fr
# test python
num = 0
start = time.time()
for i in range(0,100000):
    num = num + cal_force(4,np.array([2,3]),np.array([4,5]),np.array([6,7]),1,1,1,1)
print("python",num,time.time()-start)
# test C
num = 0
start = time.time()
for i in range(0,100000):
    num = num + example.cal_force(4,2,3,4,5,6,7,1,1,1,1)
print("C",num,time.time()-start)

# import numpy as np
# import matplotlib.pyplot as plt

# X = np.linspace(-np.pi, np.pi, 256, endpoint=True)
# C,S = np.cos(X), np.sin(X)

# plt.plot(X,C)
# plt.plot(X,S)

# plt.show()