/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "freertos.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "ZDT_X57_v2motor.h"
#include "drv_damiao6220.h"
#include "tim.h"
#include "spi.h"
#include "Motor.h"
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 4;
  hfdcan1.Init.ExtFiltersNbr = 4;
  hfdcan1.Init.RxFifo0ElmtsNbr = 10;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 10;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 2;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 10;
  hfdcan1.Init.TxBuffersNbr = 10;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 10;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */



extern osMessageQueueId_t motor_can_rx_queueHandle; // 6220 电机队列
extern volatile ZDT_Motor_State zdt_motor[2];       // ZDT 双电机状态数�?

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* 接收数据就会触发中断到这里来 */
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        uint8_t receivedData[8];
        FDCAN_RxHeaderTypeDef RxHeader;
        
        // 提取报文
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, receivedData) == HAL_OK)
        {
            // ==========================================
            // 标准帧分�?
            // ==========================================
            if (RxHeader.IdType == FDCAN_STANDARD_ID)
            {
                uint32_t id = RxHeader.Identifier;

                // 6220 云台电机反馈
                if (id == M6220_MST_ID)
                {
                    osMessageQueuePut(motor_can_rx_queueHandle, &receivedData, 0, 0);
                }
                // 底盘 M3508 左前轮反�?(ID = 0x201)
                if (id == 0x201)
                {
                    Lf_data.angle = (receivedData[0] << 8) | receivedData[1];
                    Lf_data.rpm   = (receivedData[2] << 8) | receivedData[3];
                }
                // 底盘 M3508 右前轮反�?(ID = 0x202)
                if (id == 0x202)
                {
                    Rf_data.angle = (receivedData[0] << 8) | receivedData[1];
                    Rf_data.rpm   = (receivedData[2] << 8) | receivedData[3];
                }
                // 裁判系统击打计数 (ID = 0x777)
                if (id == 0x777)
                {
                    if (receivedData[0] == 2) BIG_BALL++;
                    if (receivedData[0] == 3) SMALL_BALL++;
                }
            }
            // ==========================================
            // 扩展帧分�?(ZDT 步进电机)
            // ==========================================
            else if (RxHeader.IdType == FDCAN_EXTENDED_ID)
            {
                // 从扩展帧 Identifier 提取电机 ID: 发送时�?motor_id << 8
                uint8_t motor_id = (uint8_t)((RxHeader.Identifier >> 8) & 0xFF);
                // 合法性校验：支持 0x01 ~ 0xFE
                if (motor_id < 1 || motor_id > 2)
                    return;

                uint8_t idx = motor_id - 1;  // 数组索引

                // 回填 ID 到状态结构体（首次收到时写入�?
                zdt_motor[idx].motor_id = motor_id;
                zdt_motor[idx].rx_count++;    // 收到帧计�?

                // 调试捕获：记录最近一帧的�?3 字节
                zdt_motor[idx].dbg_d0 = receivedData[0];
                zdt_motor[idx].dbg_d1 = receivedData[1];
                zdt_motor[idx].dbg_d2 = receivedData[2];

                // ----- 回零状态回复帧: [0x3B, state, 0x6B] -----
                if (receivedData[0] == 0x3B)
                {
                   uint8_t state = receivedData[1];

                    // 1. 如果 Bit2 �?1，说明还在回零中
                    if ((state & 0x04) != 0) {
                        // 啥也不干，保�?is_stalled = 1，让 while 循环继续死等
                    }
                    // 2. 如果 Bit3 �?1，说明底层超时或没撞到失败了
                    else if ((state & 0x08) != 0) {
                        zdt_motor[idx].is_stalled = 2; // 赋值为 2 (代表失败)
                    }
                    // 3. 既没有在执行，也没有失败，说明彻底成功！
                    else {
                        zdt_motor[idx].is_stalled = 0; // 赋值为 0 (代表成功)
                    }
										
                }
                // ----- 位置回复�? 0x36 -----
                else if (receivedData[0] == 0x36)
                {
                    uint32_t temp_pos = ((uint32_t)receivedData[3] << 24) |
                                        ((uint32_t)receivedData[4] << 16) |
                                        ((uint32_t)receivedData[5] << 8)  |
                                        ((uint32_t)receivedData[6]);

                    if (receivedData[2] == 0x01)
                        zdt_motor[idx].real_pos = -(int32_t)temp_pos;
                    else
                        zdt_motor[idx].real_pos = (int32_t)temp_pos;
                }
                // ----- 错误返回�? [error_code, 0xEE, 0x6B] -----
                else if (receivedData[1] == 0xEE)
                {
                    zdt_motor[idx].error_code = receivedData[0];
                }
            }
        }
    }
}

  




void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

/* USER CODE END 1 */

