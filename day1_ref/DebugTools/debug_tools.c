#include "debug_tools.h"
#include "stm32h7xx_hal_def.h"
#include <string.h>

#define VOFA_MAX_CH  10   //最多10个通道

//内部串口句柄，初始化时绑定
static UART_HandleTypeDef *vofa_huart = NULL;

static const uint8_t JUSTFLOAT_TAIL[4] = {0x00,0x00,0x80,0x7F};  //float类型数据尾标识
static uint8_t vofa_tx_buf[VOFA_MAX_CH*4 + 4];  //发送缓冲区，最多10个float数据+尾标识



void Vofa_Init(UART_HandleTypeDef *huart)
{
    vofa_huart = huart;
}


void Vofa_SendFrame(uint8_t channel_count, const float *data)
{
    if (vofa_huart == NULL || channel_count == 0 || channel_count > VOFA_MAX_CH)
        return;

    if (vofa_huart->hdmatx != NULL
        && vofa_huart->hdmatx->State != HAL_DMA_STATE_READY)
        return;

    // 直接 memcpy 整个 float 数组，不再用 va_arg
    memcpy(vofa_tx_buf, data, channel_count * sizeof(float));

    // 拷贝尾帧
    memcpy(&vofa_tx_buf[channel_count * 4], JUSTFLOAT_TAIL, 4);

    uint16_t len = channel_count * 4 + 4;
    // 有DMA用DMA，没DMA阻塞发送
    if (vofa_huart->hdmatx != NULL)
        HAL_UART_Transmit_DMA(vofa_huart, vofa_tx_buf, len);
    else
        HAL_UART_Transmit(vofa_huart, vofa_tx_buf, len, HAL_MAX_DELAY);
}
void Vofa_SendFloat(float data)
{
    uint8_t data_buf[4];

    memcpy(data_buf,&data,sizeof(float));

    HAL_UART_Transmit(vofa_huart,data_buf,4,HAL_MAX_DELAY);

}

void Vofa_SendTail(void)
{
    uint8_t tail[4] ={0x00,0x00,0x80,0x7F};
    HAL_UART_Transmit(vofa_huart,tail,4,HAL_MAX_DELAY);
}

// ===================== SerialPlot / CustomFrame 协议 =====================
// 帧格式: [0xAA][0xBB] [float0 LE] ... [floatN LE]  (2B帧头, 无帧尾)

#define SP_MAX_CH   10

static UART_HandleTypeDef *sp_huart = NULL;
static uint8_t sp_tx_buf[2 + SP_MAX_CH * 4];  // AA BB + N*4

void SerialPlot_Init(UART_HandleTypeDef *huart)
{
    sp_huart = huart;
}

void SerialPlot_SendFrame(uint8_t channel_count, const float *data)
{
    if (sp_huart == NULL || channel_count == 0 || channel_count > SP_MAX_CH)
        return;

    if (sp_huart->hdmatx != NULL
        && sp_huart->hdmatx->State != HAL_DMA_STATE_READY)
        return;

    sp_tx_buf[0] = 0xAB;
   // sp_tx_buf[1] = 0xBB;
    memcpy(&sp_tx_buf[1], data, channel_count * sizeof(float));

    uint16_t len = 1 + channel_count * 4;

    if (sp_huart->hdmatx != NULL)
        HAL_UART_Transmit_DMA(sp_huart, sp_tx_buf, len);
    else
        HAL_UART_Transmit(sp_huart, sp_tx_buf, len, HAL_MAX_DELAY);
}
