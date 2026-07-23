#include "main.h"
#include "ZDT_X57_v2motor.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "string.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern volatile ZDT_Motor_State zdt_motor[2];

/* H7 FlashWord 写入要求 32 字节对齐，必须放文件级作用域，
   不能放函数内（armcc 禁止 auto 变量对齐超过 8 字节） */
static __align(32) uint32_t flash_data_buf[8];

uint8_t ArmorPlate_Homing(uint8_t ID){
    uint16_t timeout_cnt = 0;
    uint8_t idx = ID - 1;  // 数组索引: 电机1→[0], 电机2→[1]
    zdt_motor[idx].is_stalled = 1;// 先标记忙，防止其他任务误发位置指令
    osDelay(200); // 等待电机状态更新
    ZDT_Trigger_Collision_Homing(ID);  // 触发碰撞回正指令
    while (zdt_motor[idx].is_stalled == 1)
    {   
        ZDT_Request_Homing_Status(ID);
        osDelay(10);
        timeout_cnt++;
        if (timeout_cnt >= 3400) {//这里的时间可能还要修改
            printf("Motor %d homing TIMEOUT, dbg=[%02X %02X %02X]\r\n",
                   ID, zdt_motor[idx].dbg_d0, zdt_motor[idx].dbg_d1, zdt_motor[idx].dbg_d2);
            ZDT_Force_Stop_Homing(ID);
            zdt_motor[idx].is_stalled = 0; // 清理标志位
            return 0;
        }
    }
    if (zdt_motor[idx].is_stalled == 2)
    {
        printf("Motor %d homing FAIL, dbg=[%02X %02X %02X]\r\n",
               ID, zdt_motor[idx].dbg_d0, zdt_motor[idx].dbg_d1, zdt_motor[idx].dbg_d2);
        zdt_motor[idx].is_stalled = 0; // 恢复清白之身，方便下次使用
        return 0; // 驱动器报了失败，返回 0
    }
    // is_stalled == 0: ISR 已确认回零成功
    printf("Motor %d homing OK, dbg=[%02X %02X %02X]\r\n",
           ID, zdt_motor[idx].dbg_d0, zdt_motor[idx].dbg_d1, zdt_motor[idx].dbg_d2);
    zdt_motor[idx].is_stalled = 0;   // 清除堵转标志

    return 1;
}

/**
  * @brief  主动向电机发送：读取实时位置的请求 (0x36)
  * @param  motor_id: 电机ID (默认一般是 1)
  */
void ZDT_Request_Position(uint8_t motor_id)
{
    uint8_t TxData[2]; 
    FDCAN_TxHeaderTypeDef Txheader;
    
    // 1. FDCAN 扩展帧报文头配置
    Txheader.Identifier = motor_id << 8;         // 地址左移8位 (0x0100)
    Txheader.IdType = FDCAN_EXTENDED_ID;         // 必须使用扩展帧
    Txheader.TxFrameType = FDCAN_DATA_FRAME;     // 数据帧
    Txheader.DataLength = FDCAN_DLC_BYTES_2;     // 索要指令的数据载荷只有 2 个字节
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;       // 兼容传统 CAN 格式我现在还是发现，他还是卡在最大回正时
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 2. 组装发送指令 (严格对照说明书: 0x36 + 0x6B)
    TxData[0] = 0x36; // 命令功能码：读取实时位置
    TxData[1] = 0x6B; // 校验字节

    // 3. 推入 FDCAN 发送 FIFO 队列
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}
  

/**
  * @brief  梯形曲线位置模式 (0xFD) — 匹配官方 ZDT V2 协议
  * @param  motor_id:   电机ID
  * @param  target_pos: 目标位置 (单位: 角度°, 例 360 = 1 圈)
  * @param  max_speed:  最大速度 (单位: RPM)
  * @param  acc:        加/减速度 (单位: RPM/s)
  */
void ZDT_ArmorPlate_Move(uint8_t motor_id, int32_t target_pos)
{
    uint8_t TxData[8];
    FDCAN_TxHeaderTypeDef Txheader;

    // 反转软限位：不超过 -18 圈（正转靠堵转自然停止）
    if (target_pos < -ARMOR_RAW_MAX) target_pos = -ARMOR_RAW_MAX;

    // 内置默认速度/加速度
    uint16_t max_speed = 300;   // RPM
    uint8_t  acc       = 30;    // RPM/s

    // 1. 公共报文头
    Txheader.IdType = FDCAN_EXTENDED_ID;
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 2. 参数解析与限幅
    // 在绝对模式下，这里的 dir 代表目标坐标的“正负号”
    uint8_t dir = (target_pos < 0) ? 0x01 : 0x00;   
    
    // 提取纯绝对值 (用三目运算符最安全，不用担心 ABS 宏没定义)
    uint32_t abs_pos = (target_pos < 0) ? (uint32_t)(-target_pos) : (uint32_t)(target_pos);
    
    uint32_t vel = max_speed * 10;
    if(vel > 65535) vel = 65535; // 限制在两字节内

    // ==========================================
    // Frame 0: ID=(addr<<8)|0, DLC=8
    // ==========================================
    Txheader.Identifier = (motor_id << 8) + 0;
    Txheader.DataLength = FDCAN_DLC_BYTES_8;

    TxData[0] = 0xFD;                        // 【修复1】：必须和第二帧保持一致，使用 0xFD
    TxData[1] = dir;                         // 坐标符号
    TxData[2] = (uint8_t)(vel >> 8);         // 最大速度高
    TxData[3] = (uint8_t)(vel & 0xFF);       // 最大速度低
    TxData[4] = acc;                         // 加速度档位
    TxData[5] = (uint8_t)(abs_pos >> 24);    // 脉冲数 [31:24]
    TxData[6] = (uint8_t)(abs_pos >> 16);    // 脉冲数 [23:16]
    TxData[7] = (uint8_t)(abs_pos >> 8);     // 脉冲数 [15:8]

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);

    // ==========================================
    // Frame 1: ID=(addr<<8)|1, DLC=5 
    // ==========================================
    Txheader.Identifier = (motor_id << 8) + 1;
    Txheader.DataLength = FDCAN_DLC_BYTES_5; 

    TxData[0] = 0xFD;                        // 保持 0xFD
    TxData[1] = (uint8_t)(abs_pos & 0xFF);   // 脉冲数 [7:0]
    TxData[2] = 0x01;                        // 【修复2】：改为 0x01，彻底启用绝对位置模式！
    TxData[3] = 0x00;                        // 多机同步 (0=不启用)
    TxData[4] = 0x6B;                        // 校验码

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);

    // 发完命令后轮询位置，让 ISR 及时更新 real_pos
    for (uint8_t i = 0; i < 3; i++) {
        ZDT_Request_Position(motor_id);
        osDelay(5);
    }
}

/**
  * @brief  控制装甲板：速度模式 (FDCAN 单包发送，无需拆包)
  * @param  motor_id: 电机ID (默认1)
  * @param  speed:    目标速度 (带符号，单位 RPM，内部自动 ×10)
  * @param  acc:      加速度 (uint16_t，单位 RPM/s)
  */
void ZDT_Velocity_Mode(uint8_t motor_id, int32_t speed, uint16_t acc)
{       
    uint8_t TxData[7];
    FDCAN_TxHeaderTypeDef Txheader;
    
    // 1. FDCAN 扩展帧报文头配置
    Txheader.Identifier = motor_id << 8;         // 地址转移到 ID (例如 0x0100)
    Txheader.IdType = FDCAN_EXTENDED_ID;         // CAN 通讯帧类型固定为扩展帧
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_7;     // 数据长度为 7 字节
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 2. 解析正负号和绝对值
    uint8_t sign = (speed < 0) ? 0x01 : 0x00;    // 00表示正转，01表示反转
    uint32_t abs_speed = ABS(speed) * 10;        // 取速度绝对值并放大 10 倍 (对应0.1RPM分辨率)
    
    // 安全限幅保护 (防止传入的参数过大导致高位截断)
    if(abs_speed > 65535) {
        abs_speed = 65535; // 速度最大只能占用2个字节 (0xFFFF)
    }
    if(acc > 255) {
        acc = 255;         // 加速度档位最大为255 (0xFF)
    }

    // 3. 数据拆包组装 (严格对照成功示例：F6 01 05 DC 0A 00 6B)
    TxData[0] = 0xF6;                            // Byte 0: 速度模式功能码
    TxData[1] = sign;                            // Byte 1: 符号方向
    TxData[2] = (uint8_t)(abs_speed >> 8);       // Byte 2: 速度高 8 位
    TxData[3] = (uint8_t)(abs_speed & 0xFF);     // Byte 3: 速度低 8 位
    TxData[4] = (uint8_t)acc;                    // Byte 4: 加速度档位 (仅占1字节)
    TxData[5] = 0x00;                            // Byte 5: 多机同步标志 (填0)
    TxData[6] = 0x6B;                            // Byte 6: 校验字节 (按你要求固定为 6B)

    // 4. 发送报文
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

/**
  * @brief  电机使能/失能控制
  * @param  motor_id: 电机ID
  * @param  state: 1=使能, 0=失能
  */
void ZDT_Enable_Motor(uint8_t motor_id, uint8_t state)
{
    uint8_t TxData[5];
    FDCAN_TxHeaderTypeDef Txheader;

    Txheader.Identifier = motor_id << 8;
    Txheader.IdType = FDCAN_EXTENDED_ID;
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_5;
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    TxData[0] = 0xF3;       // 使能/失能功能码
    TxData[1] = 0xAB;       // 固定命令字
    TxData[2] = state;      // 0=失能, 1=使能
    TxData[3] = 0x00;       // snF: 多机同步标志 (0=不启用)
    TxData[4] = 0x6B;       // 校验字节

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

/**
  * @brief  【极简版】同时保存 1号 和 2号 电机的位置
  * @param  pos1: 1号电机的位置
  * @param  pos2: 2号电机的位置
  */
void Save_Both_Motors_To_Flash(int32_t pos1, int32_t pos2)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    // 关全局中断：擦写同 Bank Flash 时总线会硬停顿，
    // ISR 里如果读 Flash 会 fault，FDCAN 也会丢帧
    __disable_irq();

    // 全部填充 0xFF（擦除态），然后填两个位置值
    memset(flash_data_buf, 0xFF, sizeof(flash_data_buf));
    flash_data_buf[0] = (uint32_t)pos1;
    flash_data_buf[1] = (uint32_t)pos2;

    HAL_FLASH_Unlock();

    // 擦除扇区
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.Sector        = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK
        || SectorError != 0xFFFFFFFFU)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return;
    }

    // 256bit FlashWord 写入，flash_data 必须 32 字节对齐
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                      ADDR_FLASH_SECTOR_7, (uint32_t)flash_data_buf);

    HAL_FLASH_Lock();
    __enable_irq();
}

/**
  * @brief  从 Flash 读取指定 ID 的位置
  * @param  motor_id: 1 或者 2
  */
int32_t Read_Pos_From_Flash(uint8_t motor_id)
{
    uint32_t raw;

    if (motor_id == 1)
    {
        raw = *(__IO uint32_t *)(ADDR_FLASH_SECTOR_7);
    }
    else if (motor_id == 2)
    {
        raw = *(__IO uint32_t *)(ADDR_FLASH_SECTOR_7 + 4);
    }
    else
    {
        return 0;
    }

    // 未写入过（全 0xFF = 擦除态），返回 0 而不是 -1
    if (raw == 0xFFFFFFFFU)
        return 0;

    return (int32_t)raw;
}

/**
  * @brief  触发驱动器内置的【多圈无限位碰撞回零】
  * @param  motor_id : 电机地址
  */
void ZDT_Trigger_Collision_Homing(uint8_t motor_id)
{
    uint8_t TxData[4]; 
    FDCAN_TxHeaderTypeDef Txheader;
    
    Txheader.Identifier = motor_id << 8;         
    Txheader.IdType = FDCAN_EXTENDED_ID;         
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_4;     // 数据长度为 4 字节
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 组装触发指令: 0x9A + 回零模式 + 同步标志 + 0x6B
    TxData[0] = 0x9A; // 触发回零功能码
    TxData[1] = 0x02; // 0x02 = 触发多圈无限位碰撞回零
    TxData[2] = 0x00; // 不启用多机同步
    TxData[3] = 0x6B; // 校验字节

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

/**
  * @brief  请求获取回零状态标志位
  * @param  motor_id : 电机地址
  */
void ZDT_Request_Homing_Status(uint8_t motor_id)
{
    uint8_t TxData[2]; 
    FDCAN_TxHeaderTypeDef Txheader;
    
    Txheader.Identifier = motor_id << 8;         
    Txheader.IdType = FDCAN_EXTENDED_ID;         
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_2;     // 长度为 2 字节
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 组装请求指令: 0x3B + 0x6B
    TxData[0] = 0x3B; // 读取回零状态标志位功能码
    TxData[1] = 0x6B; // 校验字节

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

/**
  * @brief  强制中断并退出回零操作
  * @param  motor_id : 电机地址
  */
void ZDT_Force_Stop_Homing(uint8_t motor_id)
{
    uint8_t TxData[3]; 
    FDCAN_TxHeaderTypeDef Txheader;
    
    // 1. FDCAN 扩展帧报文头配置
    Txheader.Identifier = motor_id << 8;         // 地址左移8位进入 ID
    Txheader.IdType = FDCAN_EXTENDED_ID;         // 必须为扩展帧[cite: 1]
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = FDCAN_DLC_BYTES_3;     // 有效数据为 3 个字节
    Txheader.FDFormat = FDCAN_CLASSIC_CAN;
    Txheader.BitRateSwitch = FDCAN_BRS_OFF;
    Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Txheader.MessageMarker = 0x00;

    // 2. 组装发送指令 (严格对照说明书: 0x9C + 0x48 + 0x6B)[cite: 1]
    TxData[0] = 0x9C; // 命令功能码：强制中断并退出回零[cite: 1]
    TxData[1] = 0x48; // 固定参数[cite: 1]
    TxData[2] = 0x6B; // 校验字节，按你要求固定为 0x6B

    // 3. 推入 FDCAN 发送 FIFO 队列
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txheader, TxData);
}

