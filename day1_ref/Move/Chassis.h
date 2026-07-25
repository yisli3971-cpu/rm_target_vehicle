#ifndef CHASSIS_H_
#define CHASSIS_H_

#define CHASSIS_MAX_RPM  480.0f      //最大转速（rpm）
#define CHASSIS_WHEEL_BASE  0.25f    //左右两轮间距


void Chassis_Movement(float vx,float vR,float *out_lf_rpm,float *out_rf_rpm);

#endif
