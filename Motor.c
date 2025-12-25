#include "Motor.h"
#include "Pwm.h"

void motor_control(Motor_Typedef *motor, MotorState state, uint8_t speed)
{
	switch(state)
	{
		case MOTOR_STOP:
			HAL_GPIO_WritePin(motor->io_port1, motor->io_pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->io_port2, motor->io_pin2, GPIO_PIN_RESET);
			pwm_set_duty(motor->tim, motor->tim_channel, 0);
			break;
		case MOTOR_CW:
			HAL_GPIO_WritePin(motor->io_port1, motor->io_pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->io_port2, motor->io_pin2, GPIO_PIN_RESET);
			pwm_set_duty(motor->tim, motor->tim_channel, speed);
			break;
		case MOTOR_CCW:
			HAL_GPIO_WritePin(motor->io_port1, motor->io_pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->io_port2, motor->io_pin2, GPIO_PIN_SET);
			pwm_set_duty(motor->tim, motor->tim_channel, speed);
			break;
	}
}


void motor_init(Motor_Typedef *motor,
                GPIO_TypeDef *io_port1, uint16_t io_pin1,
                GPIO_TypeDef *io_port2, uint16_t io_pin2,
                TIM_HandleTypeDef *tim, uint32_t tim_channel)
{
    motor->io_port1 = io_port1;
    motor->io_pin1 = io_pin1;
    motor->io_port2 = io_port2;
    motor->io_pin2 = io_pin2;
    motor->tim = tim;
    motor->tim_channel = tim_channel;

    HAL_TIM_PWM_Start(tim, tim_channel);
}