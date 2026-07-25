#include "IBUS.h"
#include <string.h>
#include "usart.h"

volatile Remote_t Remote;

uint8_t DBUS_RX_Buffer[18];

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

/* 错误回调：当 UART 发生校验错/帧错/噪声/溢出时被调用
 * 必须在错误发生后重新启动 DMA，否则接收永久停止 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        /* 重新启动 DMA 接收，恢复数据流 */
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
        }
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, DBUS_RX_Buffer, 18);
    }
}
