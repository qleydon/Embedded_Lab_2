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
	state_at_position,		// use a separate integer to record the current position (0 through 5) for each servo.
	state_unknown,
	state_moving,
	state_waiting,
	state_recipe_ended
} ;

enum events
{
	user_entered_pause,
	user_entered_continue,
	user_entered_right,
	user_entered_left,
	user_entered_no,
	user_entered_begin,
	recipe_ended
} ;

#endif /* INC_SERVO_ENUMS_H_ */

