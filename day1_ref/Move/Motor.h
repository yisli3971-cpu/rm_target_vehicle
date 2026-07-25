#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_MAX_CURRENT  16384


//控制周期(秒), 200Hz = 0.005s
#define CONTROL_DT 0.005F

typedef enum{
   MOTOR_MODE_SPEED = 0,   //速度环
   MOTOR_MODE_ANGLE = 1     //角度环

}Motor_Mode;

typedef struct{
    int16_t angle;   //转子机械角度 (0-8191)
    int16_t rpm;     //转子转速 (rpm)
} DJMotor_Feedback;

typedef struct{

    float now_angle;   //当前输出轴角度
    float now_omega;   //当前输出轴角速度
    float now_current;  //当前电流值
    int32_t total_encoder;   //累计编码器值（多圈）
    int32_t total_round;     //累计圈数  
    uint16_t pre_encoder;     //上一拍的单圈编码器值（过零检测用）

}Motor_Data;



extern float debug_lf_current;
extern float debug_rf_current;
extern int   debug_can_tx_ret;
extern Motor_Data motor_lf_data;
extern Motor_Data motor_rf_data;
extern volatile DJMotor_Feedback Lf_data;   // 0x201 CAN 反馈
extern volatile DJMotor_Feedback Rf_data;   // 0x202 CAN 反馈

void Motor_Init(void);
void Motor_Control(void);
void Motor_Stop(void);
int  Motor_SendCurrent(int32_t lf_current,int32_t rf_current);
void Motor_SetSpeedPID(float kp,float ki,float kd);

//设定控制模式
void Motor_SetMode(Motor_Mode mode);

//设定目标速度（rpm）
void Motor_SetTargetRPM(int32_t rpm);
void Motor_SetTargetRPM_LF(int32_t rpm);
void Motor_SetTargetRPM_RF(int32_t rpm);
//设定目标角度（rad）
void Motor_SetTargetAngle(float angle_rad);

//供 VOFA 调试：读取当前目标转速
int32_t Motor_GetTargetRPM_LF(void);
int32_t Motor_GetTargetRPM_RF(void);

#ifdef __cplusplus
}
#endif

#endif
