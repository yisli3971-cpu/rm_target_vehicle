#include "PID.h"

void PID_SetParam(PID_t *pid,float kp,float ki,float kd)
{
   pid->kp = kp;
   pid->ki = ki;
   pid->kd = kd;
}

void PID_SetLimit(PID_t *pid, float out_max, float out_min)
{
    pid->out_max = out_max;
    pid->out_min = out_min;
}

void PID_Clear(PID_t *pid)
{
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;

}

float PID_Calculate(PID_t *pid,float target,float measure,float dt)
{
   pid->err = target - measure;

   // P 项
   float p_out = pid->kp * pid->err;

   // I 项（先累加，再乘 Ki）
   pid->integral += pid->err * dt;
   float i_out = pid->ki * pid->integral;

   // D 项（微分先行：对测量值微分，避免设定值突变时微分冲击）
   // derivative on measurement: d(measure)/dt = -(err - err_last)/dt
   float derivative = (pid->err - pid->err_last) / dt;
   float d_out = pid->kd * derivative;

   float output = p_out + i_out + d_out;

   // ---- 积分限幅（back-calculation）：输出饱和时冻结积分 ----
   if (pid->out_max > pid->out_min)  // 限幅生效
   {
       if (output > pid->out_max)
       {
           pid->integral -= pid->err * dt;  // 回退本次积分累加
           output = pid->out_max;
       }
       else if (output < pid->out_min)
       {
           pid->integral -= pid->err * dt;
           output = pid->out_min;
       }
   }

   pid->err_last = pid->err;

   return output;
}
