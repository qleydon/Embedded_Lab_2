/*
 * status.h
 *
 *  Created on: Sep 28, 2023
 *      Author: Quinn Leydon
 */

#ifndef INC_SERVO_ENUMS_H_
#define INC_SERVO_ENUMS_H_

enum status
{
	status_running,
	status_paused,
	status_command_error,
	status_nested_error
} ;

enum states
{
	state_at_position,
	state_unknown,
	state_moving,
	state_waiting,
	state_recipe_ended
} ;

#endif /* INC_SERVO_ENUMS_H_ */
