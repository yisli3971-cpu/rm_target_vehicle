#include "Motor.h"
#include "PID.h"
#include "fdcan.h"

//内部变量(static 仅在本文件可见)
static PID_t pid_lf_omega;
static PID_t pid_lf_angle;
static PID_t pid_rf_omega;
static PID_t pid_rf_angle;



static Motor_Mode motor_mode = MOTOR_MODE_SPEED;


static int32_t target_rpm = 0;   //目标转速(rpm)
static float target_angle = 0.0f;     //目标角度（rad）
static int32_t target_rpm_lf = 0;
static int32_t target_rpm_rf = 0;


float debug_lf_current = 0.0f;
float debug_rf_current = 0.0f;
int   debug_can_tx_ret = 0;       // CAN TX 返回值诊断

Motor_Data motor_lf_data = {0};
Motor_Data motor_rf_data = {0};


//引用Init.cpp里定义的反馈数据(声明在 Init.h)


#define ENCODER_PER_ROUND 8192   //单圈编码器刻度
#define GEARBOX_RATE    19.0F   //3508电机的减速比
#define PI          3.14159265f
#define  RPM_TO_RADPS    0.104719755F     //1rpm = 2*PI/60 rad/s


static void Motor_UpdateData(Motor_Data *data,uint16_t raw_encoder,int16_t raw_omega)
{
    //过零检测，追踪圈数
    int16_t delta = (int16_t)(raw_encoder - data->pre_encoder);

    if ( delta < -ENCODER_PER_ROUND / 2 )
    {
        //正方向过零：编码器从8000+跳回100-
        data->total_round++;
    }
    else if(delta > ENCODER_PER_ROUND / 2)
    {
        //反方向过零：编码器从100-跳到8000+
        data->total_round--;

    }

    //计算总编码器值
    data->total_encoder = data->total_round * ENCODER_PER_ROUND + raw_encoder;

    //换算为输出轴角度
    data->now_angle =((float)data->total_encoder / (float)ENCODER_PER_ROUND)*2.0f *PI /GEARBOX_RATE;
    
    //换算为输出轴角速度
    data->now_omega = (float)raw_omega *RPM_TO_RADPS / GEARBOX_RATE;

    //保存本次编码器值为下次使用
    data->pre_encoder = raw_encoder;

}

void Motor_Init(void)
{
    // 速度环 PID：
    PID_SetParam(&pid_lf_omega, 10.0f, 0.0f, 0.0015f);
    PID_SetLimit(&pid_lf_omega, MOTOR_MAX_CURRENT, -MOTOR_MAX_CURRENT);
    PID_Clear(&pid_lf_omega);

    PID_SetParam(&pid_rf_omega, 10.0f, 0.0f, 0.0015f);
    PID_SetLimit(&pid_rf_omega, MOTOR_MAX_CURRENT, -MOTOR_MAX_CURRENT);
    PID_Clear(&pid_rf_omega);

    // 角度环 PID
    PID_SetParam(&pid_lf_angle, 5.0f, 0.0f, 0.0f);
    PID_SetLimit(&pid_lf_angle, 200.0f, -200.0f);     // 角度环输出限幅 ±200 rad/s
    PID_Clear(&pid_lf_angle);

    PID_SetParam(&pid_rf_angle, 5.0f, 0.0f, 0.0f);
    PID_SetLimit(&pid_rf_angle, 200.0f, -200.0f);
    PID_Clear(&pid_rf_angle);
}


void Motor_Control(void)
{
    Motor_UpdateData(&motor_lf_data,(uint16_t)Lf_data.angle,Lf_data.rpm);
    Motor_UpdateData(&motor_rf_data,(uint16_t)Rf_data.angle,Rf_data.rpm);

    float lf_current,rf_current;

    if(motor_mode == MOTOR_MODE_SPEED)
    {
        //速度模式，速度环（target 是输出轴 RPM，反馈是电机转子 RPM，需统一单位）
        lf_current = PID_Calculate(&pid_lf_omega,(float)(target_rpm_lf * (int32_t)GEARBOX_RATE),(float)Lf_data.rpm,CONTROL_DT);
        rf_current = PID_Calculate(&pid_rf_omega,(float)(target_rpm_rf * (int32_t)GEARBOX_RATE),(float)Rf_data.rpm,CONTROL_DT);
    }
    else  // MOTOR_MODE_ANGLE
    {
        // 角度模式：外环角度 PID → 内环速度 PID
        float target_omega_lf = PID_Calculate(&pid_lf_angle, target_angle,motor_lf_data.now_angle, CONTROL_DT);
        float target_omega_rf = PID_Calculate(&pid_rf_angle, target_angle,motor_rf_data.now_angle, CONTROL_DT);

        // 把角度环输出的目标速度（rad/s）转成 rpm 给速度环
        // rad/s → rpm: × 60 / (2*PI) = × 9.5493，再 × 减速比
        float target_rpm_lf = target_omega_lf * 60.0f / (2.0f * PI) * GEARBOX_RATE;
        float target_rpm_rf = target_omega_rf * 60.0f / (2.0f * PI) * GEARBOX_RATE;

        lf_current = PID_Calculate(&pid_lf_omega, target_rpm_lf,(float)Lf_data.rpm, CONTROL_DT);
        rf_current = PID_Calculate(&pid_rf_omega, target_rpm_rf, (float)Rf_data.rpm, CONTROL_DT);
    }
    if(lf_current > MOTOR_MAX_CURRENT)  lf_current = MOTOR_MAX_CURRENT;
    if(lf_current < -MOTOR_MAX_CURRENT) lf_current = -MOTOR_MAX_CURRENT;
    if(rf_current > MOTOR_MAX_CURRENT)  rf_current = MOTOR_MAX_CURRENT;
    if(rf_current < -MOTOR_MAX_CURRENT) rf_current = -MOTOR_MAX_CURRENT;
    
    debug_lf_current = lf_current;
    debug_rf_current = rf_current;

    Motor_SendCurrent((int32_t)lf_current, (int32_t)rf_current);
}

void Motor_SetMode(Motor_Mode mode)
{
    motor_mode = mode;
    // 切换模式时清零 PID，防止积分残留
    PID_Clear(&pid_lf_omega);
    PID_Clear(&pid_rf_omega);
    PID_Clear(&pid_lf_angle);
    PID_Clear(&pid_rf_angle);
}

void Motor_SetTargetRPM(int32_t rpm)
{
    target_rpm = rpm;
    target_rpm_lf = rpm;
    target_rpm_rf = -rpm;   // 右轮与左轮反向
}
void Motor_SetTargetRPM_LF(int32_t rpm) 
{
     target_rpm_lf = rpm; 
}
void Motor_SetTargetRPM_RF(int32_t rpm)
{
    target_rpm_rf = rpm;   // 直接设定，方向由上层（Chassis）决定
}

void Motor_SetTargetAngle(float angle_rad)
{
    target_angle = angle_rad;
}


void Motor_Stop(void)
{
   Motor_SendCurrent(0 , 0);

}

void Motor_SetSpeedPID(float kp,float ki,float kd)
{
    PID_SetParam(&pid_lf_omega, kp, ki, kd);
    PID_Clear(&pid_lf_omega);
    PID_SetParam(&pid_rf_omega, kp, ki, kd);
    PID_Clear(&pid_rf_omega);
}

int Motor_SendCurrent(int32_t lf_current, int32_t rf_current)
{
    uint8_t tx_data[8];
    FDCAN_TxHeaderTypeDef tx_header = {0};

    //M3508  电流报文（can id 0x200）,大端序：
    //data[0:1] = 电机1 电流（对应反馈0x201）
    //data[2:3] = 电机2 电流（对应反馈0x202）
    //data[4:5] = 电机3
    //data[6:7] = 电机4

    tx_data[0] = (lf_current >> 8) & 0xff;
    tx_data[1] =  lf_current       & 0xff;
    tx_data[2] = (rf_current >> 8) & 0xff;
    tx_data[3] =  rf_current       & 0xff;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    tx_header.Identifier = 0x200;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = 8;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}



// ---- 供 VOFA 调试用的 getter ----
int32_t Motor_GetTargetRPM_LF(void) 
{ 
    return target_rpm_lf; 
}

int32_t Motor_GetTargetRPM_RF(void)
{ 
    return target_rpm_rf; 
}

int32_t Motor_GetTargetRPM(void)
{
    return target_rpm;
}
