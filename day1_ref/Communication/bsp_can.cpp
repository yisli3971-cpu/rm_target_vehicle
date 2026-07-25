#include "bsp_can.h"



CanManageObject g_can1_manage_object ={0};
uint8_t g_can1_0x200_tx_data[8];

int can_init(FDCAN_HandleTypeDef *hcan, CanCallback callback_function)
{
    if (hcan->Instance == FDCAN1)
    {
        g_can1_manage_object.can_handler = hcan;
        g_can1_manage_object.callback_function = callback_function;

        if (can_filter_mask_config(hcan,
                CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0) != HAL_OK)
            return -1;
        if (can_filter_mask_config(hcan,
                CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0) != HAL_OK)
            return -1;
    }

    if (HAL_FDCAN_Start(hcan) != HAL_OK)
        return -1;

    if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return -1;
    if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
        return -1;

    return 0;
}

int can_filter_mask_config(FDCAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id)
{
    FDCAN_FilterTypeDef filter = {0};

    uint8_t filter_index = object_para >> 3;
    uint8_t fifo_select = (object_para >> 2) & 0x01;
    uint8_t id_type_flag = (object_para >> 1) & 0x01;
    uint8_t frame_type   = object_para & 0x01;

    filter.FilterIndex = filter_index;
    filter.IdType = id_type_flag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = (fifo_select == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1  = id ;
    filter.FilterID2   = mask_id;

    return HAL_FDCAN_ConfigFilter(hcan, &filter);
}

uint8_t can_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = id;
    tx_header.IdType    = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1) {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                &g_can1_manage_object.rx_buffer.header,
                g_can1_manage_object.rx_buffer.data);

            g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1) {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                &g_can1_manage_object.rx_buffer.header,
                g_can1_manage_object.rx_buffer.data);

            g_can1_manage_object.callback_function(&g_can1_manage_object.rx_buffer);
        }
    }
}
