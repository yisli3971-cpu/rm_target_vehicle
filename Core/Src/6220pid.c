#include "6220pid.h"
#include "drv_damiao6220.h"   // V_MAX/V_MIN/T_MAX/T_MIN + FDCAN_Motor_MoveState
#include "cmsis_os2.h"
#include "FreeRTOS.h"

extern FDCAN_Motor_MoveState motor_movestate;

static float g_target_velocity = 0.0f;
static float g_target_position = 0.0f;
static PID_Mode_t g_mode = PID_MODE_POSITION;

static float vel_integral = 0.0f;
static float filtered_vel = 0.0f;

void clear_velocity_pid(void)
{
    vel_integral = 0.0f;
    filtered_vel = 0.0f;
}

void set_pid_mode(PID_Mode_t mode){ g_mode = mode; }
PID_Mode_t get_pid_mode(void){ return g_mode; }
void set_position(float pos){ g_target_position = pos; }
void set_velocity(float vel){ g_target_velocity = vel; }

/* ====== 位置环 (外环): 200Hz, 位置误差 → 目标速度 ====== */
float update_pid_p(void)
{
    static uint32_t last_tick = 0;
    uint32_t now_tick = osKernelGetTickCount();
    float dt = (now_tick - last_tick) * 0.001F;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
    last_tick = now_tick;

    if (g_mode != PID_MODE_POSITION)
        return 0.0f;

    static uint8_t pos_cnt = 0;
    pos_cnt++;
    if (pos_cnt < POS_LOOP_DIV)
        return 0.0f;   // 没到位置环周期，不更新 g_target_velocity
    pos_cnt = 0;

    float d_pos = g_target_position - motor_movestate.gimbal_pos;

    /* 最短路径：过半圈则反向 */
    float half_range = (P_MAX - P_MIN) * 0.5f;
    if (d_pos > half_range)       d_pos -= (P_MAX - P_MIN);
    else if (d_pos < -half_range) d_pos += (P_MAX - P_MIN);

    float target_velocity = p_kp * d_pos;

    if (target_velocity > V_MAX) target_velocity = V_MAX;
    if (target_velocity < V_MIN) target_velocity = V_MIN;

    g_target_velocity = target_velocity;

    /* 位置微分 (阻尼): 误差变化率 → 抑制过冲 */
    static float last_d_pos = 0.0f;
    float d_pos_deriv = (d_pos - last_d_pos) / dt;
    last_d_pos = d_pos;
    float damping = p_kd * d_pos_deriv;

    /* 位置前馈: 误差 → 直接扭矩加成, 绕过速度环动态 */
    float ff_torque = p_ff * d_pos + damping;
    if (ff_torque > T_MAX)  ff_torque = T_MAX;
    if (ff_torque < T_MIN)  ff_torque = T_MIN;
    return ff_torque;
}

/* ====== 速度环 (内环): 1kHz, 速度误差 → 扭矩 ====== */
float update_pid_v(void)
{
    static uint32_t last_tick = 0;
    uint32_t now_tick = osKernelGetTickCount();
    float dt = (now_tick - last_tick) * 0.001F;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
    last_tick = now_tick;

    filtered_vel = 0.6f * filtered_vel + 0.4f * motor_movestate.gimbal_vel;

    float now_error = g_target_velocity - filtered_vel;
    float fabs_tv  = g_target_velocity > 0 ? g_target_velocity : -g_target_velocity;
    float fabs_err = now_error > 0 ? now_error : -now_error;

    /* 目标速度接近零 → 清积分，防止停车抖动 */
    if (fabs_tv < 0.1f) {
        vel_integral = 0.0f;
    }

    /* 误差死区：< 0.5 rad/s 不累积积分 */
    if (fabs_err >= 0.5f) {
        vel_integral += v_ki * now_error * dt;
    }

    if (vel_integral > T_MAX) vel_integral = T_MAX;
    if (vel_integral < T_MIN) vel_integral = T_MIN;

    float torque = v_kp * now_error + vel_integral;

    /* 摩擦力仅在速度较大时补偿 */
    if (fabs_tv > 2.0f) {
        torque += Feedforward_Friction(g_target_velocity);
    }

    if (torque > T_MAX) torque = T_MAX;
    if (torque < T_MIN) torque = T_MIN;

    return torque;
}

float get_target_velocity(void) { return g_target_velocity; }

/* 随机速度生成: LCG 伪随机, 范围 [-30, -10] ∪ [10, 30] rad/s */
float generate_random_velocity(unsigned int *seed) {
    *seed = *seed * 1103515245 + 12345;
    int r = (int)(*seed & 0x7FFF) % 42;  // 0~41
    if (r < 21)
        return -30.0f + (float)r;         // [-30, -10]
    else
        return 10.0f + (float)(r - 21);   // [10, 30]
}

float Feedforward_Friction(float target_speed) {
    float Tc = 0.05f;
    if (target_speed > 0.01f)        return  Tc;
    else if (target_speed < -0.01f)  return -Tc;
    else                             return 0.0f;
}
