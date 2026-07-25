#include "Chassis.h"

void Chassis_Movement(float vx,float vR,float *out_lf_rpm,float *out_rf_rpm)
{
    float forward_speed = vx * CHASSIS_MAX_RPM;
    float rotation_speed = vR * CHASSIS_MAX_RPM;

    // 差速驱动：左轮 = 前进 + 旋转，右轮 = 前进 - 旋转
    // 右轮物理安装方向与左轮相反，因此右轮输出取反
    *out_lf_rpm = forward_speed + rotation_speed;
    *out_rf_rpm = -(forward_speed - rotation_speed);
}
