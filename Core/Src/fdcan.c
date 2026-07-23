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
extern volatile ZDT_Motor_State zdt_motor[2];       // ZDT 双电机状态数组

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
            // 分拣通道 1：标准帧 (你的 6220 电机)
            // ==========================================
            if (RxHeader.IdType == FDCAN_STANDARD_ID)
            {
                // 只收 6220 反馈帧 (ID = MST_ID)
                if (RxHeader.Identifier == M6220_MST_ID)
                {
                    osMessageQueuePut(motor_can_rx_queueHandle, &receivedData, 0, 0);
                }
            }
            // ==========================================
            // 分拣通道 1。1：标准帧 (装甲板)
            // ==========================================
            if(RxHeader.IdType == 0x777){
                if(receivedData[0]==2){
                    BIG_BALL++;
                }
                if(receivedData[0]==3){
                    SMALL_BALL++;
                }

            }
            // ==========================================
            // 分拣通道 2：扩展帧 (张大头闭环步进电机)
            // ==========================================
            else if (RxHeader.IdType == FDCAN_EXTENDED_ID)
            {
                // 从扩展帧 Identifier 提取电机 ID: 发送时是 motor_id << 8
                uint8_t motor_id = (uint8_t)((RxHeader.Identifier >> 8) & 0xFF);
                // 合法性校验：支持 0x01 ~ 0xFE
                if (motor_id < 1 || motor_id > 2)
                    return;

                uint8_t idx = motor_id - 1;  // 数组索引

                // 回填 ID 到状态结构体（首次收到时写入）
                zdt_motor[idx].motor_id = motor_id;
                zdt_motor[idx].rx_count++;    // 收到帧计数

                // 调试捕获：记录最近一帧的前 3 字节
                zdt_motor[idx].dbg_d0 = receivedData[0];
                zdt_motor[idx].dbg_d1 = receivedData[1];
                zdt_motor[idx].dbg_d2 = receivedData[2];

                // ----- 回零状态回复帧: [0x3B, state, 0x6B] -----
                if (receivedData[0] == 0x3B)
                {
                   uint8_t state = receivedData[1];

                    // 1. 如果 Bit2 为 1，说明还在回零中
                    if ((state & 0x04) != 0) {
                        // 啥也不干，保持 is_stalled = 1，让 while 循环继续死等
                    }
                    // 2. 如果 Bit3 为 1，说明底层超时或没撞到失败了
                    else if ((state & 0x08) != 0) {
                        zdt_motor[idx].is_stalled = 2; // 赋值为 2 (代表失败)
                    }
                    // 3. 既没有在执行，也没有失败，说明彻底成功！
                    else {
                        zdt_motor[idx].is_stalled = 0; // 赋值为 0 (代表成功)
                    }
										
                }
                // ----- 位置回复帧: 0x36 -----
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
                // ----- 错误返回帧: [error_code, 0xEE, 0x6B] -----
                else if (receivedData[1] == 0xEE)
                {
                    zdt_motor[idx].error_code = receivedData[0];
                }
            }
        }
    }
}

    




//void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan)
/*这个问题问得极其敏锐！你发现了一个很多人看 HAL 库源码时都会懵圈的盲点。

我直接给你吃颗定心丸：**在你目前的配置下，这个 `HighPriorityMessageCallback` 绝对不会被调用！它现在就是一段永远不会执行的“死代码”。**

为了搞懂它们之间的联系，我们需要把 FDCAN 内部的“物流分拣系统”拆开来看。

在 FDCAN 硬件内部，对于接收到的数据，它其实提供了**两条完全不同的通道**：

### 通道一：普通快递传送带（FIFO 0 和 FIFO 1）

这就是你刚才用 `FDCAN_IT_RX_FIFO0_NEW_DATA` 激活的那个通道。

* **逻辑：** 外部发来的报文，默认都会被扔进 FIFO 0 这个大箱子里排队（最多可以排 3 个或更多，看你的配置）。
* **触发的函数：** 只要箱子里进了新东西，就会触发我们之前聊的 `HAL_FDCAN_RxFifo0Callback`。
* **应用场景：** 大多数普通的通信。比如你在机械系统里，各个节点在以 100Hz 的频率不停地汇报电机的当前转速、底盘的坐标、传感器的温度。这些数据按顺序排队处理就行。

### 通道二：VIP 紧急红色通道（High Priority Message）

这就是你刚才问到的那个神秘回调函数的用武之地。

* **逻辑：** 在工业控制中，总有一些信息是**绝对不能排队**的。比如某个电机驱动器发来了一个 `ID = 0x001` 的“紧急急停（E-Stop）”**或**“过流烧毁警告”。如果这个救命的消息被扔进了 FIFO 0，排在几十个普通的温度数据后面，等 CPU 处理到它的时候，机器可能已经炸了。
* **它的特权：** FDCAN 硬件允许你在底层设置一个“安检过滤器（Filter）”。你可以告诉硬件：“只要看到 ID 是 `0x001` 的报文，**不要把它扔进 FIFO 排队！** 给我直接把它送到‘高优先级寄存器’里！”
* **触发的函数：** 当这种 VIP 报文到达时，它会跳过普通的收件箱，直接触发你问的这个 `HAL_FDCAN_HighPriorityMessageCallback`。

---

### 如何才能让这个函数被调用？

如果你未来的项目需要用到这个“紧急通道”，你需要做两件事，它才会被“唤醒”：

**第一步：设置硬件过滤器（Filter），把某个 ID 设为 VIP。**
你需要调用 `HAL_FDCAN_ConfigFilter()`，在配置结构体中，把某个特定 ID 的过滤动作设为存入专门的 Rx Buffer，或者直接标记为高优先级。

**第二步：专门为它开启一个“紧急通知”。**
就像你之前用 `ActivateNotification` 开启 FIFO 中断一样，你需要再加一行代码，专门开启高优先级中断：

```c
// 开启高优先级报文到达的专属打断权限
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_HIGH_PRIO_MESSAGE, 0);

```

只要你不写这两步，这根“VIP 红色电话线”就是断开的，那个 `HighPriorityMessageCallback` 就永远是一潭死水。

### 总结你们之间的联系

* **`HAL_FDCAN_RxFifo0Callback`** = 处理日常工作汇报的常规部门。
* **`HAL_FDCAN_HighPriorityMessageCallback`** = 处理突发致命故障的应急指挥部。

这就好比在设计一套复杂的机械自动化总线时，你既需要常规的物流通道，也必须预留紧急制动通道。

你现在手头的板子之间，准备互相发送什么样的数据格式呢？是准备自己定义一套协议，把传感器数据打包发过去吗？*/
  




void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

/* USER CODE END 1 */

