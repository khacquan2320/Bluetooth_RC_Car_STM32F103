#include "Pwm.h"
//duty: 0->100
void pwm_set_duty(TIM_HandleTypeDef *tim, uint32_t chanel,uint8_t duty)
{
	uint16_t ccr = (uint16_t)duty*(tim->Instance->ARR)/100;
	switch(chanel)
	{
		case TIM_CHANNEL_1:
			tim->Instance->CCR1 = ccr;
			break;
		case TIM_CHANNEL_2:
			tim->Instance->CCR2 = ccr;
			break;
		case TIM_CHANNEL_3:
			tim->Instance->CCR3 = ccr;
			break;
		case TIM_CHANNEL_4:
			tim->Instance->CCR4 = ccr;
			break;
	}
}
