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
#include "Motor.h"
#include "Chassis.h"
#include "IBUS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern FDCAN_Motor_MoveState motor_movestate;       // 6220 电机状态（单例）
volatile ZDT_Motor_State zdt_motor[2];              // ZDT 双电机状态: [0]=电机1, [1]=电机2
volatile ZDT_Cmd_Msg_t zdt_cmd[2];                   // ZDT 双电机命令接收
extern volatile uint8_t g_control_tick;              // TIM6 1kHz 控制节拍
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(x) ((x) < 0 ? -(x) : (x))
/* 遥控器通道到 6220 云台的映射 */
#define CH2_POS_SCALE   (P_MAX / 660.0f)    // ch2 → 位置
#define CH3_VEL_SCALE   (V_MAX / 660.0f)    // ch3 → 速度
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
/* Definitions for myTask06_3508 */
osThreadId_t myTask06_3508Handle;
const osThreadAttr_t myTask06_3508_attributes = {
  .name = "myTask06_3508",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
void StartTask06(void *argument);

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

  /* creation of myTask06_3508 */
  myTask06_3508Handle = osThreadNew(StartTask06, NULL, &myTask06_3508_attributes);

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
   * 功能: 6220 云台电机 PID + 遥控器控制
   * - s1=1: 位置模式, ch2 控制位置增量
   * - s1=2: 速度模式, ch3 直接控速度
   * - s1=3: 停机, 扭矩=0
   */
  float t_ff = 0;
  PID_Mode_t current_mode = PID_MODE_VELOCITY;
  uint8_t last_s1 = 0;

  set_pid_mode(PID_MODE_VELOCITY);  // 确保默认速度模式
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, DBUS_RX_Buffer, 18);   // 启动 DBUS
  osDelay(200);                      // 等首帧 IBUS 数据到达

  for(;;)
  {
    /* 遥控器模式切换: s1=1 位置, s1=2 速度, s1=3 停机 */
    if (Remote.s1 != last_s1) {
      last_s1 = Remote.s1;
      if (Remote.s1 == 1)      { set_pid_mode(PID_MODE_POSITION); current_mode = PID_MODE_POSITION; }
      else if (Remote.s1 == 2) { set_pid_mode(PID_MODE_VELOCITY); current_mode = PID_MODE_VELOCITY; }
    }

    /* s2=1: ZDT 全部回零校准 (边沿触发仅一次) */
    {
      static uint8_t last_s2 = 0;
      if (Remote.s2 == 1 && last_s2 != 1) {
        zdt_motor[0].calibration = 1;
        zdt_motor[1].calibration = 1;
      }
      last_s2 = Remote.s2;
    }

    if (Remote.s1 == 3) {
      t_ff = 0.0f;   // 停机: 扭矩清零
    } else {
      if (current_mode == PID_MODE_POSITION) {
        /* 增量式位置: ch2 控制位置变化速率, 摇杆归零停在原位 */
        static float accum_pos = 0.0f;
        static uint8_t pos_inited = 0;
        if (!pos_inited) { accum_pos = motor_movestate.gimbal_pos; pos_inited = 1; }
        float rate = (float)Remote.ch2 * 0.000009f;
        if (rate > -0.0003f && rate < 0.0003f) rate = 0.0f;  // 死区
        accum_pos += rate;
        if (accum_pos > P_MAX)  accum_pos = P_MAX;
        if (accum_pos < P_MIN)  accum_pos = P_MIN;
        set_position(accum_pos);
      } else {
        /* 速度模式: ch3 → 目标速度 */
        float vel = (float)Remote.ch3 * CH3_VEL_SCALE;
        if (vel > -1.0f && vel < 1.0f) vel = 0.0f;  // 死区
        set_velocity(vel);
      }

      osMutexAcquire(key_6220Handle, osWaitForever);
      t_ff = motor_ready ? update_pid() : 0.0f;
      osMutexRelease(key_6220Handle);
    }

    DRV6220_Control_Torque(0x01, t_ff, current_mode);
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
   * - 查询两个电机的实时位置 (0x36)
   * - 位移 ≥ 3600 脉冲 (一圈) → 写入 Flash
   * - ZDT 查询和 LCD 刷新间隔 1s，避免 SPI 冲突
   * - LCD 每周期刷新一次击打数
   */
  /* Infinite loop */
 int32_t last_pos[2];
  last_pos[0] = Read_Pos_From_Flash(1);
  last_pos[1] = Read_Pos_From_Flash(2);
  for(;;)
  {
        ZDT_Request_Position(1);
        osDelay(5);
        ZDT_Request_Position(2);
        osDelay(5);

        int32_t diff1 = ABS(zdt_motor[0].real_pos - last_pos[0]);
        int32_t diff2 = ABS(zdt_motor[1].real_pos - last_pos[1]);

        if (diff1 >= 3600 || diff2 >= 3600)
        {
            Save_Both_Motors_To_Flash(zdt_motor[0].real_pos, zdt_motor[1].real_pos);
            last_pos[0] = zdt_motor[0].real_pos;
            last_pos[1] = zdt_motor[1].real_pos;
        }

        // ZDT 查询和 LCD 刷新隔开
        osDelay(1000);
        LCD_DisplayUpdate();
    osDelay(10000);  // LCD 刷新间隔 10s
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

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06_3508 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /**
   * Task6: myTask06_3508 (osPriorityNormal)
   * 功能: M3508 底盘电机遥控控制
   * - 轮询 g_control_tick (TIM6 1kHz 节拍)
   * - 每拍读取 IBUS 遥控器 → 差速运动学 → 更新目标转速
   * - Motor_Control() 由 TIM6 ISR 直接调用 (PID + CAN 发送)
   */
  Motor_SetMode(MOTOR_MODE_SPEED);

  uint8_t last_tick = 0;

  for(;;)
  {
    if (g_control_tick == last_tick) {
      osDelay(1);
      continue;
    }
    last_tick = g_control_tick;

    /* s2=1: ZDT 校准模式，暂停 3508 底盘控制 */
    if (Remote.s2 != 1) {
      float vx = (float)Remote.ch1 / 660.0f;
      float vR = (float)Remote.ch0 / 660.0f;
      float lf_rpm, rf_rpm;
      Chassis_Movement(vx, vR, &lf_rpm, &rf_rpm);
      Motor_SetTargetRPM_LF((int32_t)lf_rpm);
      Motor_SetTargetRPM_RF((int32_t)rf_rpm);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

