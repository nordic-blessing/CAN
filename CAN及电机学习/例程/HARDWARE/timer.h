#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f4xx.h"

void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri);

void TIM_Delayms(TIM_TypeDef * TIMx, uint32_t DelayMs);
void TIM_Delayus(TIM_TypeDef * TIMx, uint16_t Delayus);
void TIM_Delay100us(TIM_TypeDef * TIMx, uint16_t Delay100us);

void TIM1_PWM_Init(u32 arr,u32 psc);//µç´Å·§¿ª¹Ø

#endif 

