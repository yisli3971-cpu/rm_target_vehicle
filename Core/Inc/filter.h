#ifndef __FILTER_H_
#define __FILTER_H_

//一阶低通滤波

typedef struct
{
    float alpha;
    float last_out;
} FirstOrder_LPF_t;

void LPF_Init(FirstOrder_LPF_t *lpf , float alpha);

float LPF_Calc(FirstOrder_LPF_t *lpf, float input);

#endif
