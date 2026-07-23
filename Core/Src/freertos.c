/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_damiao6220.h"
#include "fdcan.h"
#include "6220pid.h"
#include "ZDT_X57_v2motor.h"
#include "tim.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern FDCAN_Motor_MoveState motor_movestate;       // 6220 电机状态（单例）
volatile ZDT_Motor_State zdt_motor[2];              // ZDT 双电机状态: [0]=电机1, [1]=电机2
volatile ZDT_Cmd_Msg_t zdt_cmd[2];                   // ZDT 双电机命令接收
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(x) ((x) < 0 ? -(x) : (x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t motor_ready = 0;  // 0=未就绪, 1=反馈已到达, PID 可以跑
/* USER CODE END Variables */
/* Definitions for MotorReceiveTas */
osThreadId_t MotorReceiveTasHandle;
const osThreadAttr_t MotorReceiveTas_attributes = {
  .name = "MotorReceiveTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorSendTask */
osThreadId_t MotorSendTaskHandle;
const osThreadAttr_t MotorSendTask_attributes = {
  .name = "MotorSendTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ZDT_Receive_Sen */
osThreadId_t ZDT_Receive_SenHandle;
const osThreadAttr_t ZDT_Receive_Sen_attributes = {
  .name = "ZDT_Receive_Sen",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Flash_Save_Task */
osThreadId_t Flash_Save_TaskHandle;
const osThreadAttr_t Flash_Save_Task_attributes = {
  .name = "Flash_Save_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ZDT_Control */
osThreadId_t ZDT_ControlHandle;
const osThreadAttr_t ZDT_Control_attributes = {
  .name = "ZDT_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motor_can_rx_queue */
osMessageQueueId_t motor_can_rx_queueHandle;
const osMessageQueueAttr_t motor_can_rx_queue_attributes = {
  .name = "motor_can_rx_queue"
};
/* Definitions for key_6220 */
osMutexId_t key_6220Handle;
const osMutexAttr_t key_6220_attributes = {
  .name = "key_6220"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of key_6220 */
  key_6220Handle = osMutexNew(&key_6220_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motor_can_rx_queue */
  motor_can_rx_queueHandle = osMessageQueueNew (16, 8, &motor_can_rx_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorReceiveTas */
  MotorReceiveTasHandle = osThreadNew(StartDefaultTask, NULL, &MotorReceiveTas_attributes);

  /* creation of MotorSendTask */
  MotorSendTaskHandle = osThreadNew(StartTask02, NULL, &MotorSendTask_attributes);

  /* creation of ZDT_Receive_Sen */
  ZDT_Receive_SenHandle = osThreadNew(StartTask03, NULL, &ZDT_Receive_Sen_attributes);

  /* creation of Flash_Save_Task */
  Flash_Save_TaskHandle = osThreadNew(StartTask04, NULL, &Flash_Save_Task_attributes);

  /* creation of ZDT_Control */
  ZDT_ControlHandle = osThreadNew(StartTask05, NULL, &ZDT_Control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the MotorReceiveTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /**
   * Task1: MotorReceiveTas (osPriorityHigh)
   * 功能: 6220 电机 CAN 反馈接收
   * - 上电使能 FDCAN 中断，使能电机
   * - 收第一帧反馈初始化 gimbal_pos
   * - 循环阻塞读取消息队列，解析反馈帧
   */
  /* Infinite loop */
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
  uint8_t rx_data[8];
	osDelay(300);   
      DRV6220_Enable_Motor(0x01);
  

  /* 3. 收第一帧反馈 → 初始化真实位置 */
  osMessageQueueGet(motor_can_rx_queueHandle, rx_data, NULL, osWaitForever);
  osMutexAcquire(key_6220Handle, osWaitForever);
  if ((rx_data[0] & 0x0F) == 0x01) {
      Prase_Motor_Feedback(rx_data);
  }
  osMutexRelease(key_6220Handle);

  motor_ready = 1;                     // 4. 通知 MotorSendTask 可以开始 PID

  for(;;)
  {
    
    if(osMessageQueueGet(motor_can_rx_queueHandle, rx_data, NULL, osWaitForever) == osOK)
    {
      osMutexAcquire(key_6220Handle, osWaitForever);
      // 反馈帧 D[0] 低4位 = 电机ID，0x01 = 电机1
      if((rx_data[0] & 0x0F) == 0x01){
          Prase_Motor_Feedback(rx_data);
      }
      osMutexRelease(key_6220Handle);
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the MotorSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /**
   * Task2: MotorSendTask (osPriorityNormal)
   * 功能: 6220 电机 PID 控制
   * - motor_ready=1 后开始 PID 运算
   * - updata_pid() 计算扭矩，DRV6220_Control_Torque 发送
   * - PID 模式由 set_pid_mode() 决定，扭矩映射自动跟随
   */
  float t_ff=0;
  set_pid_mode(PID_MODE_VELOCITY);
  for(;;)
  {
     if (!motor_ready) { osDelay(1); continue; }

     osMutexAcquire(key_6220Handle, osWaitForever);
     t_ff = updata_pid();
     DRV6220_Control_Torque(0x01, t_ff, get_pid_mode());
     osMutexRelease(key_6220Handle);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ZDT_Receive_Sen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /**
   * Task3: ZDT_Receive_Sen (osPriorityNormal)
   * 功能: 1号 ZDT 步进电机 使能 + 回零校准 + 位置控制
   * - 上电使能、从 Flash 恢复位置
   * - calibration=1 → 执行碰撞回零 (最多 3 次重试)
   * - changepos=1 且空闲 → 执行绝对位置运动
   * - 成功: real_pos=0, calibration=0
   */
  /* Infinite loop */
  ZDT_Enable_Motor(1, 1);
  zdt_motor[0].motor_id    = 1;
  zdt_motor[0].calibration = 0;
  zdt_motor[0].is_stalled  = 0;
  zdt_motor[0].real_pos    = Read_Pos_From_Flash(1);
  zdt_motor[0].changepos   = 0;

  for(;;)
  {
      /* ---- 1号电机回零校准 ---- */
      if (zdt_motor[0].calibration == 1) {
          uint8_t ok = 0;
          for (uint8_t attempt = 0; attempt < 3; attempt++) {
              uint8_t r = ArmorPlate_Homing(1);
              if (r == 1) {
                  printf("Motor 1 calibration OK\r\n");
                  zdt_motor[0].real_pos  = 0;
                  zdt_motor[0].calibration = 0;
                  ok = 1;
                  break;
              }
              if (attempt < 2) {
                  osDelay(5000);
                  ZDT_Force_Stop_Homing(1);
              }
          }
          if (!ok) {
              printf("Motor 1 calibration FAIL\r\n");
              osDelay(1000);
          }
      }

      /* ---- 1号电机位置控制 ---- */
      if (zdt_motor[0].changepos == 1) {
          if (zdt_motor[0].is_stalled != 1) {
              ZDT_ArmorPlate_Move(1, zdt_cmd[0].target_val);
              zdt_motor[0].changepos = 0;
          }
      }

      osDelay(10);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Flash_Save_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /**
   * Task4: Flash_Save_Task (osPriorityBelowNormal)
   * 功能: ZDT 位置查询 + Flash 存储 + LCD 击打数刷新
   * - 独立查询两个电机的实时位置 (0x36)
   * - 任一电机位移 ≥ 3600 脉冲 (一圈) → 写入 Flash (双电机同时保存)
   * - ZDT 查询和 LCD 刷新间隔 1s，避免 SPI 冲突
   * - LCD 每周期刷新一次击打数
   */
  /* Infinite loop */
  int32_t last_pos[2];
  last_pos[0] = Read_Pos_From_Flash(1);
  last_pos[1] = Read_Pos_From_Flash(2);

  for(;;)
  {
      /* ---- 查询 1号电机位置 ---- */
      ZDT_Request_Position(1);
      osDelay(5);

      /* ---- 查询 2号电机位置 ---- */
      ZDT_Request_Position(2);
      osDelay(5);

      /* ---- 独立判断位移，任一超阈值则双电机保存 ---- */
      int32_t diff1 = ABS(zdt_motor[0].real_pos - last_pos[0]);
      int32_t diff2 = ABS(zdt_motor[1].real_pos - last_pos[1]);

      if (diff1 >= 3600 || diff2 >= 3600)
      {
          Save_Both_Motors_To_Flash(zdt_motor[0].real_pos, zdt_motor[1].real_pos);
          last_pos[0] = zdt_motor[0].real_pos;
          last_pos[1] = zdt_motor[1].real_pos;
      }

      /* ---- LCD 刷新 (与 ZDT 查询隔开，避免 SPI 冲突) ---- */
      osDelay(1000);
      LCD_DisplayUpdate();
      osDelay(70000);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the ZDT_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /**
   * Task5: ZDT_Control (osPriorityLow)
   * 功能: 2号 ZDT 步进电机 使能 + 回零校准 + 位置控制
   * - 上电使能、从 Flash 恢复位置
   * - calibration=1 → 执行碰撞回零 (最多 3 次重试)
   * - changepos=1 且空闲 → 执行绝对位置运动
   * - 成功: real_pos=0, calibration=0
   */
  /* Infinite loop */
  ZDT_Enable_Motor(2, 1);
  zdt_motor[1].motor_id    = 2;
  zdt_motor[1].calibration = 0;
  zdt_motor[1].is_stalled  = 0;
  zdt_motor[1].real_pos    = Read_Pos_From_Flash(2);
  zdt_motor[1].changepos   = 0;

  for(;;)
  {
      /* ---- 2号电机回零校准 ---- */
      if (zdt_motor[1].calibration == 1) {
          uint8_t ok = 0;
          for (uint8_t attempt = 0; attempt < 3; attempt++) {
              uint8_t r = ArmorPlate_Homing(2);
              if (r == 1) {
                  printf("Motor 2 calibration OK\r\n");
                  zdt_motor[1].real_pos  = 0;
                  zdt_motor[1].calibration = 0;
                  ok = 1;
                  break;
              }
              if (attempt < 2) {
                  osDelay(5000);
                  ZDT_Force_Stop_Homing(2);
              }
          }
          if (!ok) {
              printf("Motor 2 calibration FAIL\r\n");
              osDelay(1000);
          }
      }

      /* ---- 2号电机位置控制 ---- */
      if (zdt_motor[1].changepos == 1) {
          if (zdt_motor[1].is_stalled != 1) {
              ZDT_ArmorPlate_Move(2, zdt_cmd[1].target_val);
              zdt_motor[1].changepos = 0;
          }
      }

      osDelay(10);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

