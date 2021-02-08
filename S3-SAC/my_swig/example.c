#include <time.h>

double My_variable = 3.0;

// # 执行器速度，质点位置，质心位置，质心速度，角速度，mu,k, 暂时不封装旋转的
float cal_force(int act_speed, float r_x, float r_y, float r_cm_x, float r_cm_y, float V_cm_X, float V_cm_y,float omega, float mu, float k, int flag)
{
    float U_r_x = act_speed;
//     rotate = [[0, 1], [-1, 0]]
//     R = np.dot([r[0]-r_cm[0], r[1]-r_cm[1]], rotate)
    float Dv = 20;                       // # 振荡上升表面分界值太大或者力力矩太大,正负振荡表明分界值太小
    float Rv = 5;
    float fr = 0;
    if (flag == 1)       //# 计算合力      ,假设速度大于Dv时与速度无关，小于Dv时与速度线性，Dv可以修正
    {
        if ((U_r_x - V_cm_X > -Dv)&&(U_r_x - V_cm_X < Dv))      //# 相对速度差比较
        {
            fr = (mu*k*(U_r_x - V_cm_X)/Dv);
        }
        else if(U_r_x > V_cm_X)
        {
            fr = (mu*k*(U_r_x - V_cm_X)/(U_r_x - V_cm_X));
        }
        else
        {
            fr = - (mu*k*(U_r_x - V_cm_X)/(U_r_x - V_cm_X));
        }
    }   
    return fr;
}


int fact(int n) {
  if (n <= 1) return 1;
  else return n*fact(n-1);
}

int my_mod(int x, int y) {
  return (x%y);
}

char *get_time()
{
  time_t ltime;
  time(&ltime);
  return ctime(&ltime);
}