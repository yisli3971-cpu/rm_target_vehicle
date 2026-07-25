#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#include "stm32h7xx_hal.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_FILTER(x)       ((x) << 3)   // 滤波器编号，占 bit3~bit7
#define CAN_FIFO_0          (0 << 2)     // 收到数据放 FIFO0
#define CAN_FIFO_1          (1 << 2)     // 收到数据放 FIFO1
#define CAN_STDID           (0 << 1)     // 标准帧 ID（11位）
#define CAN_EXTID           (1 << 1)     // 扩展帧 ID（29位）
#define CAN_DATA_TYPE       (0 << 0)     // 数据帧
#define CAN_REMOTE_TYPE     (1 << 0)     // 远程帧


typedef struct CanRxBuffer
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CanRxBuffer;


typedef void (*CanCallback)(CanRxBuffer *);

typedef struct CanManageObject
{
    FDCAN_HandleTypeDef *can_handler;
    CanRxBuffer rx_buffer;
    CanCallback callback_function;
} CanManageObject;

extern CanManageObject g_can1_manage_object;
extern uint8_t g_can1_0x200_tx_data[8];
extern FDCAN_HandleTypeDef hfdcan1;


int  can_init(FDCAN_HandleTypeDef *hcan, CanCallback callback_function);
int  can_filter_mask_config(FDCAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id);
uint8_t can_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H_ */
