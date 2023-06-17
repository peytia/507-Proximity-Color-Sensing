/*
 * motor_driver.h
 *
 *  Created on: Apr 27, 2023
 *      Author: Harrison Hirsch, Peyton Archibald
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "stdint.h"
//#include "stm324xx_hal_conf.h"
#include "main.h"

typedef struct motor_params {
	TIM_HandleTypeDef*	htim;
	uint32_t			ChannelPinA;
	uint32_t			ChannelPinB;
} motor_params_t;

void enable_channel(motor_params_t motor, uint32_t StartTimerChannel);
void disable_channel(motor_params_t motor, uint32_t StopTimerChannel);
void set_PWM(motor_params_t motor, int8_t duty);

#endif /* INC_MOTOR_DRIVER_H_ */
