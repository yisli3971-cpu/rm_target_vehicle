#include "drv_damiao6220.h"
#include <string.h>
#include "fdcan.h"
FDCAN_Motor_MoveState motor_movestate;

//发送数据
void DRV6220_Control_Torque(uint8_t motor_id, float torque,PID_Mode_t PID_MODE)
{
    uint8_t TxData[8];
    uint16_t electric=0;
    memset(TxData, 0, sizeof(TxData));
FDCAN_TxHeaderTypeDef Txheader;
  Txheader.Identifier = motor_id;
  Txheader.IdType = FDCAN_STANDARD_ID;
  Txheader.TxFrameType=FDCAN_DATA_FRAME;
  Txheader.DataLength=FDCAN_DLC_BYTES_8;
  Txheader.FDFormat=FDCAN_CLASSIC_CAN;
  Txheader.BitRateSwitch=FDCAN_BRS_OFF;
  Txheader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
  Txheader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  Txheader.MessageMarker=0x00;
  float span = (float)((1<<12)-1);//扭矩占了12位
    if(PID_MODE==PID_MODE_POSITION){
  if(torque<=T_MIN) electric= 0;
  else{
  if(torque>=T_MAX) electric =(int)span;
    else{
  electric=(torque-T_MIN)/(T_MAX-T_MIN)*span;
        }
      }
    }
    if(PID_MODE==PID_MODE_VELOCITY){
  if(torque<=-5) electric= 0;
  else{
  if(torque>=5) electric =(int)span;
    else{
  electric=(torque+5.0f)/10.0f*span;  // ±5 Nm → 0~4095
        }
      }
    }
    TxData[6]=TxData[6]|((electric>>8)&0X0f);
    TxData[7]=electric&0x00ff;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

/**
  * @brief  6220 电机使能/失能
  * @param  motor_id: 电机 ID (默认 1)
  * @param  state: 1=使能, 0=失能
  */
void DRV6220_Enable_Motor(uint8_t motor_id)
{
    uint8_t TxData[8];
    FDCAN_TxHeaderTypeDef Txheader;
    Txheader.Identifier = motor_id;
    Txheader.IdType = FDCAN_STANDARD_ID;
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_8;
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    TxData[0] = 0xFF; TxData[1] = 0xFF; TxData[2] = 0xFF; TxData[3] = 0xFF;
    TxData[4] = 0xFF; TxData[5] = 0xFF; TxData[6] = 0xFF; TxData[7] = 0xFC;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
    
}

void DRV6220_Disable_Motor(uint8_t motor_id){

    uint8_t TxData[8];
    FDCAN_TxHeaderTypeDef Txheader;
    Txheader.Identifier = motor_id;
    Txheader.IdType = FDCAN_STANDARD_ID;
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_8;
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    TxData[0] = 0xFF; TxData[1] = 0xFF; TxData[2] = 0xFF; TxData[3] = 0xFF;
    TxData[4] = 0xFF; TxData[5] = 0xFF; TxData[6] = 0xFF; TxData[7] = 0xFD;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);

}
/*位置模式发送帧*/
/*
void DRV6220_Control_PosVel(uint8_t motor_id, float p_des, float v_des)
{
    uint8_t TxData[8];
    memset(TxData, 0, sizeof(TxData));
FDCAN_TxHeaderTypeDef Txheader;
Txheader.Identifier = CMD_POS_VEL_MODE+motor_id;
  Txheader.IdType = FDCAN_STANDARD_ID;
  Txheader.TxFrameType=FDCAN_DATA_FRAME;
  Txheader.DataLength=FDCAN_DLC_BYTES_8;
  Txheader.FDFormat=FDCAN_CLASSIC_CAN;
  Txheader.BitRateSwitch=FDCAN_BRS_OFF;
  Txheader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
  Txheader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  Txheader.MessageMarker=0x00;

    memcpy(TxData, &p_des, sizeof(p_des));
    memcpy(TxData + sizeof(p_des), &v_des, sizeof(v_des));
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

//速度模式发送帧
void DRV6220_Control_Velocity(uint8_t motor_id, float v_des)
{
    uint8_t TxData[8];
    memset(TxData, 0, sizeof(TxData));
    FDCAN_TxHeaderTypeDef Txheader;
    Txheader.Identifier = CMD_VEL_MODE+motor_id;
  Txheader.IdType = FDCAN_STANDARD_ID;
  Txheader.TxFrameType=FDCAN_DATA_FRAME;
  Txheader.DataLength=FDCAN_DLC_BYTES_4;
  Txheader.FDFormat=FDCAN_CLASSIC_CAN;
  Txheader.BitRateSwitch=FDCAN_BRS_OFF;
  Txheader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
  Txheader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  Txheader.MessageMarker=0x00;

    memcpy(TxData, &v_des, sizeof(v_des));
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}
*/

//解析电机反馈数据的状态
void Prase_Motor_Feedback(uint8_t *rx_data){
    uint8_t err=(rx_data[0]&0xf0)>>4;
  switch(err){
        case 0:
            printf("失能\n");
            break;
        case 1:
            printf("使能\n");
            break;
        case 8:
            printf("超压\n");
            break;
        case 9:
            printf("欠压\n");
            break;
        case 10:
            printf("过流\n");
            break;
        case 11:
            printf("MOS 过温\n");
            break;
        case 12:
            printf("电机线圈过温\n");
            break;
        case 13:
            printf("通讯丢失\n");
            break;
        case 14:
            printf("过载\n");
            break;
        default:
            printf("未知错误\n");
            break;
    }
//解析电机反馈数据的状态
    uint16_t pos=(rx_data[1]<<8)|(rx_data[2]);
    uint16_t vel=rx_data[3]<<4|(rx_data[4]>>4);
    uint16_t torque=(rx_data[4]&0x0F)<<8|(rx_data[5]);
//将解析到的值转换为实际的物理量
    motor_movestate.gimbal_pos=((float)pos/65535.0f)*(P_MAX-P_MIN)+P_MIN;
    motor_movestate.gimbal_vel=((float)vel/4095.0f)*(V_MAX-V_MIN)+V_MIN;
    motor_movestate.gimbal_torque=((float)torque/4095.0f)*(T_MAX-T_MIN)+T_MIN;
}

