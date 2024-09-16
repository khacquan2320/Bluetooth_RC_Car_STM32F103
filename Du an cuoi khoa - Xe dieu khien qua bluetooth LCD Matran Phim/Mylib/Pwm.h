#ifndef PWM_H
#define PWM_H

#include "stm32f1xx.h"
void pwm_set_duty(TIM_HandleTypeDef *tim, uint32_t chanel,uint8_t duty);

#endif
