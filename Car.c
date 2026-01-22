#include "Car.h"
#include "Motor.h"

//float left_ratio = 1.0;
//float right_ratio = 1.0;

Motor_Typedef motor_left;
Motor_Typedef motor_right;

void car_control(CarState car_state, uint8_t speed)
{
	switch(car_state)
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
			motor_control(&motor_right, MOTOR_CW, 0.6*speed);
			break;
//		case CAR_BACKWARD_LEFT_STATE:
//			motor_control(&motor_left, MOTOR_CCW, 0.6*speed);
//			motor_control(&motor_right, MOTOR_CCW, speed);
//			break;
//		case CAR_BACKWARD_RIGHT_STATE:
//			motor_control(&motor_left, MOTOR_CCW, speed);
//			motor_control(&motor_right, MOTOR_CCW, 0.6*speed);
//			break;
		default:
			break;
	}
}


//void car_steering_control(CarState move_dir, uint8_t speed, float left_ratio, float right_ratio)
//{
//    uint8_t speed_left = (uint8_t)(speed * left_ratio);
//    uint8_t speed_right = (uint8_t)(speed * right_ratio);

//    switch(move_dir)
//    {
//        case CAR_FORWARD_STATE:
//            motor_control(&motor_left, MOTOR_CW, speed_left);
//            motor_control(&motor_right, MOTOR_CW, speed_right);
//            break;

//        case CAR_BACKWARD_STATE:
//            motor_control(&motor_left, MOTOR_CCW, speed_left);
//            motor_control(&motor_right, MOTOR_CCW, speed_right);
//            break;

//        case CAR_LEFT_STATE:   // R? trái m?m
//            motor_control(&motor_left, MOTOR_CW, speed_left);   // quay ch?m
//            motor_control(&motor_right, MOTOR_CW, speed_right); // quay nhanh
//            break;

//        case CAR_RIGHT_STATE:  // R? ph?i m?m
//            motor_control(&motor_left, MOTOR_CW, speed_left);   // quay nhanh
//            motor_control(&motor_right, MOTOR_CW, speed_right); // quay ch?m
//            break;

//        default:
//            break;
//    }
//}

void car_init(TIM_HandleTypeDef *htim)
{
	motor_init(&motor_left,  GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, htim, TIM_CHANNEL_1);
  motor_init(&motor_right, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, htim, TIM_CHANNEL_2);
	car_control(CAR_STOP_STATE, 0);
}