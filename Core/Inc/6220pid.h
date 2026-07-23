#ifndef _6220PID_H
#define _6220PID_H
// 不在此 include drv_damiao6220.h，避免循环依赖
// drv_damiao6220.h 里已包含本头文件

typedef enum {
    PID_MODE_POSITION,  //位置模式：位置环 → 速度环
    PID_MODE_VELOCITY   // 速度模式：直接速度环
} PID_Mode_t;

#define p_kp   2.0f
#define p_ki   2.0f
#define p_kd   3.0f

#define v_kp   2.0f     // 速度 P: 主力
#define v_ki   0.3f     // 速度 I: 辅助消除静差
#define v_kd   0.0f
#define v_ff   0.002f   // 加速度前馈: 够小, 主力靠 PI

void set_pid_mode(PID_Mode_t mode);
PID_Mode_t get_pid_mode(void);
void set_position(float pos);
void set_velocity(float vel);
float updata_pid(void);
float Feedforward_Friction(float target_speed);
#endif // _6220PID_H    
