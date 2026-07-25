#include "Init.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Motor.h"
#include "IBUS.h"
#include "usart.h"




volatile DJMotor_Feedback Lf_data;
volatile DJMotor_Feedback Rf_data;



void can1_callback(CanRxBuffer *CAN_RxMessage)
{
    switch ( CAN_RxMessage->header.Identifier)
{
    case 0x201:
    Lf_data.angle = (CAN_RxMessage->data[0] << 8 ) |CAN_RxMessage->data[1];
    Lf_data.rpm = (CAN_RxMessage->data[2]<< 8 ) | CAN_RxMessage->data[3];  
    break;
    
    case 0x202:
    Rf_data.angle = (CAN_RxMessage->data[0] << 8 ) |CAN_RxMessage->data[1];
    Rf_data.rpm = (CAN_RxMessage->data[2]<< 8 ) | CAN_RxMessage->data[3];
    break;
}
}




void Init(void)
{
       if (can_init(&hfdcan1, can1_callback) != 0)
       {
           /* CAN 初始化失败，检查接线/终端电阻/电机电源 */
           while(1);
       }
       Motor_Init();

       HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_Buffer,18);

}