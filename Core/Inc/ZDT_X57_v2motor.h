#ifndef __ZDT_X57_v2motor_h__
#define __ZDT_X57_v2motor_h__

#include "fdcan.h"


typedef struct {
    uint8_t motor_id;      // 电机 ID (1 或 2)，收到 CAN 帧时回填
    int32_t real_pos;      // 实时位置 (脉冲数)
    uint8_t calibration;   // 是否执行回正 (0：不执行, 1:执行)
    uint8_t is_stalled;    // 堵转/报错标志位 (0:正常, 1:堵转/报错)
    uint8_t changepos;      // 位置是否发生变化 (0:未变化, 1:已变化)
    uint16_t rx_count; 			// 收到帧的总计数（非错误帧也计）
		uint8_t	error_code;	
		uint8_t dbg_d0;
		uint8_t dbg_d1;
		uint8_t dbg_d2;
} ZDT_Motor_State;

typedef struct {
    uint8_t  motor_id;   // 控制几号电机 (通常是 1)
    int32_t  target_val; // 目标位置(放大10倍) 或 目标速度
    uint16_t speed;      // 限制的最大速度 (位置模式用)
    uint16_t acc;        // 加速度
    uint8_t  state;      // 仅限使能/失能指令使用 (1=使能, 0=失能)
} ZDT_Cmd_Msg_t;

/* 双电机全局状态数组，索引 0→电机1, 索引 1→电机2 */
extern volatile ZDT_Motor_State zdt_motor[2];

extern volatile ZDT_Cmd_Msg_t zdt_cmd[2];
//flash 的位置
#define ADDR_FLASH_SECTOR_7  ((uint32_t)0x080E0000)
/* 绝对值宏，避免拖 stdlib.h */
#define ABS(x) ((x) < 0 ? -(x) : (x))
// ==========================================
// 装甲板物理限位与换算宏定义
// ==========================================
// 规定：1圈 = 360度 = 3600 (电机底层单位)
// 用户输入：放大10倍 (比如输入 180 代表 18.0 圈，输入 55 代表 5.5 圈)

// 比例系数：用户输入的 1 个单位(0.1圈) 对应底层多少？(3600 / 10 = 360)
#define ARMOR_RATIO_X10    360  

// 软限位边界 (底层单位)
#define ARMOR_RAW_MIN      0                        // 物理最低点: 0
#define ARMOR_RAW_MAX      (18 * 3600)              // 物理最高点: 64800

uint8_t ArmorPlate_Homing(uint8_t ID); //执行回正
void ZDT_Request_Position(uint8_t motor_id);
void ZDT_ArmorPlate_Move(uint8_t motor_id, int32_t target_pos);
void ZDT_Velocity_Mode(uint8_t motor_id, int32_t speed, uint16_t acc);
void ZDT_Enable_Motor(uint8_t motor_id, uint8_t state);
void Save_Both_Motors_To_Flash(int32_t pos1, int32_t pos2);
int32_t Read_Pos_From_Flash(uint8_t motor_id);
void ZDT_Trigger_Collision_Homing(uint8_t motor_id);
void ZDT_Request_Homing_Status(uint8_t motor_id);
void ZDT_Force_Stop_Homing(uint8_t motor_id);
#endif
