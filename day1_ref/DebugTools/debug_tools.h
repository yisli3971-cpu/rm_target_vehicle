#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include "stm32h7xx_hal.h"

// ===================== VOFA / JustFloat 协议 =====================
//初始化vofa
void Vofa_Init(UART_HandleTypeDef *huart);

// 发送 JustFloat 帧：channel_count 个 float + 尾帧
// 改用数组传入，避免 va_arg 在 STM32H7 硬浮点 ABI 下的兼容问题
void Vofa_SendFrame(uint8_t channel_count, const float *data);

//发送一个float的值
void Vofa_SendFloat(float data);

//发送尾帧
void Vofa_SendTail(void);

// ===================== SerialPlot / CustomFrame 协议 =====================
// 初始化 SerialPlot（绑定串口）
void SerialPlot_Init(UART_HandleTypeDef *huart);

// 发送 CustomFrame 帧：帧头 '$' + N个float(LE) + 帧尾 '\n'
// channel_count: 通道数(≤10)，data: float 数组指针
void SerialPlot_SendFrame(uint8_t channel_count, const float *data);

#endif
