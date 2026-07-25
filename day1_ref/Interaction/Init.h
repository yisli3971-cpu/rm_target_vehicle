#ifndef INIT_H
#define INIT_H

#ifdef __cplusplus
extern "C" {
    #endif

    #include "stm32h7xx_hal.h"


    typedef struct
    {

        int16_t angle;
        int16_t rpm;

    }DJMotor_Feedback;

    /* CAN 回调中写入，FreeRTOS 任务中读取，必须 volatile */
    extern volatile DJMotor_Feedback Lf_data;
    extern volatile DJMotor_Feedback Rf_data;

    //UART 接收回调函数指针类型
    typedef void (*UartCallback)(uint8_t *buffer,uint16_t length);
    

    void Init(void);

    #ifdef __cplusplus
}
#endif

#endif /* INIT_H */