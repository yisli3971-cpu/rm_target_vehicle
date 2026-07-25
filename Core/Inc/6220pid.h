#ifndef _6220PID_H
#define _6220PID_H
// 不在此 include drv_damiao6220.h，避免循环依赖
// drv_damiao6220.h 里已包含本头文件

typedef enum {
    PID_MODE_POSITION,  //位置模式：位置环 → 速度环
    PID_MODE_VELOCITY   // 速度模式：直接速度环
} PID_Mode_t;

/* 位置环参数 (200Hz) */
#define p_kp   2.0f
#define p_ki   2.0f
#define p_kd   3.0f

/* 速度环参数 (1kHz) */
#define v_kp   0.4f     // 速度 P: 低增益减少噪声放大
#define v_ki   0.05f    // 速度 I: 仅在误差大时累积
#define v_kd   0.0f

/* 位置环分频: 每 POS_LOOP_DIV 次调用执行一次位置环 (200Hz @ 1kHz) */
#define POS_LOOP_DIV  5

void set_pid_mode(PID_Mode_t mode);
PID_Mode_t get_pid_mode(void);
void set_position(float pos);
void set_velocity(float vel);
float update_pid(void);
float Feedforward_Friction(float target_speed);
#endif // _6220PID_H
