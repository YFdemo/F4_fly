#ifndef __PWM_OUTPUT_H
#define	__PWM_OUTPUT_H

#include "stm32f10x.h"

void TIM5_PWM_Init(void);
void TIM5_PWM_OUTPUT(u16 DR1,u16 DR2,u16 DR3,u16 DR4);
void TIM4_PWM_Init(void);
void TIM4_PWM_OUTPUT(u16 DR3,u16 DR4);

#endif /* __PWM_OUTPUT_H */

