#ifndef CAR_H
#define CAR_H

#include "stm32f1xx_hal.h"

typedef enum
{
	CAR_STOP_STATE,
	CAR_FORWARD_STATE,
	CAR_BACKWARD_STATE,
	CAR_FORWARD_LEFT_STATE,
	CAR_FORWARD_RIGHT_STATE,
//	CAR_BACKWARD_LEFT_STATE,
//	CAR_BACKWARD_RIGHT_STATE
}CarState;

void car_init(TIM_HandleTypeDef *htim);
void car_control(CarState car_state, uint8_t speed);

#endif
