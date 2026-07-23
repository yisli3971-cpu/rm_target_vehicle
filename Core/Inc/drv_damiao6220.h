#ifndef __DRV_DAMIAO6220_H
#define __DRV_DAMIAO6220_H

#include "main.h"
#include "usart.h"
#include "fdcan.h"
#define CMD_POS_VEL_MODE 0x100
#define CMD_VEL_MODE 0x200

/* 6220 反馈帧 CAN ID (MST_ID)，调试助手可配，出厂默认 0 */
#define M6220_MST_ID  0x01

#define P_MIN -12.5f   // 位置最小值 (弧度)
#define P_MAX 12.5f    // 位置最大值 (弧度)
#define V_MIN -45.0f   // 速度最小值 (弧度/秒)
#define V_MAX 45.0f    // 速度最大值 (弧度/秒)
#define T_MIN -18.0f   // 扭矩最小值 (牛米，注意协议规定可能大于电机实际峰值2.7)
#define T_MAX 18.0f    // 扭矩最大值 (牛米)



typedef struct{
  float gimbal_pos;
  float gimbal_vel;
  float gimbal_torque;
} FDCAN_Motor_MoveState;

#include "6220pid.h"   // 在结构体定义之后 include，供外部使用 PID_Mode_t

void DRV6220_Control_PosVel(uint8_t motor_id, float p_des, float v_des);
void DRV6220_Control_Velocity(uint8_t motor_id, float v_des);
void Prase_Motor_Feedback(uint8_t *rx_data);
void DRV6220_Control_Torque(uint8_t motor_id, float torque,PID_Mode_t PID_MODE);
void DRV6220_Enable_Motor(uint8_t motor_id);
void DRV6220_Disable_Motor(uint8_t motor_id);
#endif /* __DRV_DAMIAO6220_H */
