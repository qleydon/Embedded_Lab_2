/*
 * SERVO_cfg.c
 *
 *  Created on: Sep 29, 2023
 *      Author: Quinn Leydon
 */

#include "SERVO.h"
#include "main.h"

SERVO_Cfg SERVO_Cfg_1= // Servo Motor 1 Configurations
{
	GPIOA,
	56, //TIM3_CH1,
	TIM3,
	&TIM3->CCR1,
	TIM_CHANNEL_1,
	1000,
	0.65,
	2.3,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	status_paused,
	state_waiting
};

SERVO_Cfg SERVO_Cfg_2= // Servo Motor 2 Configurations
{
    GPIOA,
	38,//TIM3_CH2,
    TIM3,
    &TIM3->CCR2,
    TIM_CHANNEL_2,
    1000,
    0.65,
    2.3,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	status_paused,
	state_waiting
};
