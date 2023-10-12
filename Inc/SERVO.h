/*
 * SERVO.h
 *
 *  Created on: Sep 29, 2023
 *      Author: Quinn Leydon
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "Servo_enums.h"
#include "main.h"

typedef struct
{
    const GPIO_TypeDef * SERVO_GPIO;
    const uint16_t       SERVO_PIN;
    const TIM_TypeDef*   TIM_Instance;
    const uint32_t*      TIM_CCRx;
    const uint32_t       PWM_TIM_CH;
    const uint32_t       TIM_CLK;
    float          MinPulse;
    float          MaxPulse;
    uint8_t		   		 SERVO_POSITION;
    uint8_t		   		 SERVO_POSITION_GOAL;
    uint8_t		  		 SERVO_COUNT_MOVE;
	uint8_t		  		 SERVO_COUNT_LOOP;
	uint8_t		  		 SERVO_COUNT_WAIT;
	uint8_t		  		 LOOP_START;
	uint8_t		  		 LOOP_FLAG;
	uint8_t		  		 INDEX;
	uint8_t				 RECIPE_INDEX;
    enum status	   		 SERVO_STATUS;
    enum states		     SERVO_STATE;


}SERVO_Cfg;


extern SERVO_Cfg SERVO_Cfg_1;
extern SERVO_Cfg SERVO_Cfg_2;

#endif /* INC_SERVO_H_ */
