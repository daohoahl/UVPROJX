#include "Car.h"
#include "Motor.h"

Motor_Typedef motor_left;
Motor_Typedef motor_right;

CarState car_state = CAR_STOP_STATE;

void car_control(CarState input_state, uint8_t speed)
{
	car_state = input_state;
	
	switch(input_state)
	{
		case CAR_STOP_STATE:
			motor_control(&motor_left, MOTOR_STOP, 0);
			motor_control(&motor_right, MOTOR_STOP, 0);
			break;
		case CAR_FORWARD_STATE:
			motor_control(&motor_left, MOTOR_CW, speed);
			motor_control(&motor_right, MOTOR_CW, speed);
			break;
		case CAR_BACKWARD_STATE:
			motor_control(&motor_left, MOTOR_CCW, speed);
			motor_control(&motor_right, MOTOR_CCW, speed);
			break;
		case CAR_FORWARD_LEFT_STATE:
			motor_control(&motor_left, MOTOR_CW, 0.6*speed);
			motor_control(&motor_right, MOTOR_CW, speed);
			break;
		case CAR_FORWARD_RIGHT_STATE:
			motor_control(&motor_left, MOTOR_CW, speed);
			motor_control(&motor_right, MOTOR_CW, 0.5*speed);
			break;
//		case CAR_BACKWARD_LEFT_STATE:
//			motor_control(&motor_left, MOTOR_CCW, 0.45*speed);
//			motor_control(&motor_right, MOTOR_CCW, speed);
//			break;
//		case CAR_BACKWARD_RIGHT_STATE:
//			motor_control(&motor_left, MOTOR_CCW, speed);
//			motor_control(&motor_right, MOTOR_CCW, 0.45*speed);
//			break;
		default:
			break;
	}
}

void car_init(TIM_HandleTypeDef *htim)
{
	motor_init(&motor_left,  GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, htim, TIM_CHANNEL_1);
  motor_init(&motor_right, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, htim, TIM_CHANNEL_2);
	car_control(CAR_STOP_STATE, 0);
}