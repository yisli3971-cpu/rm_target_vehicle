#include "6220pid.h"
#include "drv_damiao6220.h"   // V_MAX/V_MIN/T_MAX/T_MIN + FDCAN_Motor_MoveState
#include "cmsis_os2.h"
#include "FreeRTOS.h"

extern FDCAN_Motor_MoveState motor_movestate;  // extern 放 .c 里，避免头文件循环依赖

static float g_target_velocity = 0.0f; //两个函数之间的传参
static float g_target_position = 0.0f;
static PID_Mode_t g_mode =PID_MODE_POSITION;

void set_pid_mode(PID_Mode_t mode){  //接口：修改模式
g_mode=mode;
}

PID_Mode_t get_pid_mode(void){
    return g_mode;
}

void set_position(float pos){g_target_position = pos;}   //接口：设置位置  pos
void set_velocity(float vel){g_target_velocity = vel;}   //接口：设置速度  v


float updata_pid(){

    //实际去获取dt
    static uint32_t last_tick =0;
    uint32_t now_tick =osKernelGetTickCount();
    float dt = (now_tick-last_tick) * 0.001F;
    if(dt<=0.0f||dt > 0.1f) dt=0.001f;
    last_tick = now_tick;
    //判断是否需要位置
    
    if(g_mode == PID_MODE_POSITION ){
        float d_pos=g_target_position - motor_movestate.gimbal_pos;

        float half_range = (P_MAX - P_MIN) * 0.5f;
        if (d_pos > half_range) {
             d_pos -= (P_MAX - P_MIN);  // 正向误差超过半圈，减去一圈，改为反向转动
        } 
         else if (d_pos < -half_range) {
              d_pos += (P_MAX - P_MIN);  // 反向误差超过半圈，加上一圈，改为正向转动
                            }
                            
        float target_velocity=p_kp*d_pos;
 
        g_target_velocity=target_velocity;
        
        if(g_target_velocity>V_MAX) g_target_velocity=V_MAX;
        if(g_target_velocity<V_MIN) g_target_velocity=V_MIN;

    }

    //速度环 PI + 加速度前馈
    static float vel_integral = 0.0f;
    static float last_target = 0.0f;      // 上一次速度目标，用于算加速度

    //一阶低通滤波
    static float filtered_vel = 0.0f;
    filtered_vel = 0.7f * filtered_vel + 0.3f * motor_movestate.gimbal_vel;

    float now_error = g_target_velocity - filtered_vel;

    // 加速度前馈：目标速度变化 → 预估扭矩，不等 PI 反应
    float target_accel = (g_target_velocity - last_target) / dt;
    last_target = g_target_velocity;

    vel_integral += v_ki * now_error * dt;
    if (vel_integral > T_MAX) vel_integral = T_MAX;
    if (vel_integral < T_MIN) vel_integral = T_MIN;

    float torque = v_kp * now_error + vel_integral + v_ff * target_accel;

    /* 摩擦补偿：速度目标 > 0.5 rad/s 时加，接近零时 PI 自稳 */
    float fabs_tv = g_target_velocity > 0 ? g_target_velocity : -g_target_velocity;
    if (fabs_tv > 0.5f) {
        torque += Feedforward_Friction(g_target_velocity);
    }

    if (torque > T_MAX) torque = T_MAX;
    if (torque < T_MIN) torque = T_MIN;

    return torque;

}

float Feedforward_Friction(float target_speed) {
    float Tc = 0.15f;

    if (target_speed > 0.01f)        return  Tc;
    else if (target_speed < -0.01f)  return -Tc;
    else                             return 0.0f;
}
