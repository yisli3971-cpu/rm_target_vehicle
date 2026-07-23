#include "filter.h"

void LPF_Init(FirstOrder_LPF_t *lpf, float alpha) 
{
    lpf->alpha = alpha;
    lpf->last_out = 0.0f; // 记忆清零
}

// 每次收到数据时，执行滤波计算
float LPF_Calc(FirstOrder_LPF_t *lpf, float input) 
{
    
    lpf->last_out = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->last_out);
    
  
    return lpf->last_out;
}
