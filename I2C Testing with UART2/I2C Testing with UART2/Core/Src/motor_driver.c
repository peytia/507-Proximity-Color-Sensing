/*
 * motor_driver.c
 *
 *  Created on: Apr 27, 2023
 *      Author: Harrison Hirsch, Peyton Archibald
 */

#include "motor_driver.h"

void enable_channel(motor_params_t motor, uint32_t StartTimerChannel)
{
	HAL_TIM_PWM_Start(motor.htim, StartTimerChannel);
}

void disable_channel(motor_params_t motor, uint32_t StopTimerChannel)
{
	HAL_TIM_PWM_Stop(motor.htim, StopTimerChannel);
}

/*void set_PWM(motor_params_t motor, uint32_t PWM_duty_PinA, uint32_t PWM_duty_PinB)
{
	__HAL_TIM_SET_COMPARE(motor.htim, motor.ChannelPinA, PWM_duty_PinA);
	__HAL_TIM_SET_COMPARE(motor.htim, motor.ChannelPinB, PWM_duty_PinB);
}*/

void set_PWM(motor_params_t motor, int8_t duty)
{
	float ScaleCoef = -4799.0/128.0;
	uint32_t dutyPinA = 4799;
	uint32_t dutyPinB = 4799;
	if (duty > 0)
	{
		dutyPinB = ScaleCoef*duty + 4799;
	}
	else if (duty < 0)
	{
		dutyPinA = -ScaleCoef*duty + 4799;
	}
	__HAL_TIM_SET_COMPARE(motor.htim, motor.ChannelPinA, dutyPinA);
	__HAL_TIM_SET_COMPARE(motor.htim, motor.ChannelPinB, dutyPinB);
}
