%module example

%{
#include "example.h"
%}

extern double My_variable;
extern int fact(int n);
extern int my_mod(int x, int y);
extern char *get_time();
extern float cal_force(int act_speed, float r_x, float r_y, float r_cm_x, float r_cm_y, float V_cm_X, float V_cm_y,float omega, float mu, float k, int flag);
