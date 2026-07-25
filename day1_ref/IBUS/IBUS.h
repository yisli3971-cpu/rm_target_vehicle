#ifndef IBUS_H_
#define IBUS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
typedef struct
{

    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;


    uint8_t s1;
    uint8_t s2;
    

}Remote_t;


/* Remote 在 DMA 中断回调中更新，FreeRTOS 任务中读取，必须 volatile */
extern volatile Remote_t Remote;
extern uint8_t DBUS_RX_Buffer[18];

/* 调试用：DMA 帧接收计数器 */
extern volatile uint32_t dbus_rx_count;
extern volatile uint32_t dbus_err1_count;
extern volatile uint32_t dbus_err2_count;
extern volatile uint32_t dbus_pe_count;
extern volatile uint32_t dbus_fe_count;
extern volatile uint32_t dbus_ne_count;
extern volatile uint32_t dbus_ore_count;
extern volatile uint32_t dbus_isr_snapshot;
extern volatile uint32_t dma1_s1_count;
extern volatile uint32_t dma1_s1_isr;

//IBUS 解析函数（回调函数调用）
void Remote_DBUS_to_RC(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
