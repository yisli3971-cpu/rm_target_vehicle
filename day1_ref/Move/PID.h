#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float err;
  float err_last;
  float integral;
  float out_max;    // 输出上限（0=不限幅，向后兼容）
  float out_min;    // 输出下限

} PID_t;

void PID_SetParam(PID_t *pid,float kp,float ki,float kd);
void PID_SetLimit(PID_t *pid,float out_max,float out_min);
void PID_Clear(PID_t *pid);
float PID_Calculate(PID_t *pid,float target,float measure,float dt);

#ifdef __cplusplus
}
#endif

#endif
