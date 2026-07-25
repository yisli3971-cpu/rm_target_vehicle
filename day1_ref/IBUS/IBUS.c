#include "IBUS.h"
#include <string.h>
#include "usart.h"

volatile Remote_t Remote;

uint8_t DBUS_RX_Buffer[18];  /* IT 模式不需要 DMA，无需特殊段 */

/* 调试计数器 */
volatile uint32_t dbus_rx_count = 0;        /* 成功接收帧数 (Size == 18) */
volatile uint32_t dbus_err1_count = 0;
volatile uint32_t dbus_err2_count = 0;       /* UART 硬件错误次数 (ErrorCallback) */
volatile uint32_t dbus_short_count = 0;     /* IDLE 回调但 Size != 18 */

void Remote_DBUS_to_RC(uint8_t *pData)
{
    
   
    
    Remote.ch0 = (((pData[0]) | (pData[1] << 8)) & 0x7FF) - 1024;
    Remote.ch1 = (((pData[1] >> 3) | (pData[2] << 5)) & 0x7FF) - 1024;
    Remote.ch2 = (((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x7FF) - 1024;
    Remote.ch3 = (((pData[4]  >> 1) | (pData[5] << 7)) & 0x7FF) - 1024;

        // 开关保留原始值
    Remote.s1 = ((pData[5] >> 4) & 0x0C) >> 2;
    Remote.s2 = (pData[5] >> 4) & 0x03;
   
 
}

/* 错误类型计数器：PE=校验错 FE=帧错 NE=噪声 ORE=溢出
 * 注意：这些计数器在 UART5_IRQHandler 中递增（HAL 处理前），
 *       因为 HAL 在调用 ErrorCallback 前会清零错误标志 */
volatile uint32_t dbus_pe_count = 0;
volatile uint32_t dbus_fe_count = 0;
volatile uint32_t dbus_ne_count = 0;
volatile uint32_t dbus_ore_count = 0;
volatile uint32_t dbus_isr_snapshot = 0;  /* 最近一次 UART5 ISR 寄存器的完整值 */
volatile uint32_t dma1_s1_count = 0;      /* DMA1 Stream1 中断次数 */
volatile uint32_t dma1_s1_isr = 0;        /* DMA1 Stream1 触发时的 LISR 值 */

/* 错误回调：当 UART 发生校验错/帧错/噪声/溢出时被调用
 * 必须在错误发生后重新启动 DMA，否则接收永久停止 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        dbus_err2_count++;  /* 调试：错误次数 +1 */

        /* 重新启动 IT 接收，恢复数据流 */
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, DBUS_RX_Buffer, 18);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if(Size == 18)
        {
            Remote_DBUS_to_RC(DBUS_RX_Buffer);
            dbus_rx_count++;  /* 调试：成功接收 +1 */
        }
        else
        {
            dbus_err1_count++;  /* 调试：帧长异常 +1 */
        }

        HAL_UARTEx_ReceiveToIdle_IT(&huart5, DBUS_RX_Buffer, 18);
    }
}
