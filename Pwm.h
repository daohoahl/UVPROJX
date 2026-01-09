#ifndef Pwm_H
#define Pwm_H

#include "stm32f1xx_hal.h"

void pwm_set_duty(TIM_HandleTypeDef *tim, uint32_t channel, uint8_t duty);

#endif
