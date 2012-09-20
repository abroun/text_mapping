//------------------------------------------------------------------------------
// This module provides control over the motors attached to the board. At the
// moment this module only offers postion control. Essentially, every frame
// the desired position and movement speed is transmitted down to the motors.
//------------------------------------------------------------------------------

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/// Number of motors
#define MOTOR_NJOINTS			20

#define MOTOR_ERROR			-1
#define MOTOR_SUCCESS			1

void MOTOR_ConfigureAllMotors();
int16_t MOTOR_SetMotorPostionAndSpeed( uint16_t id, uint16_t pos, uint16_t speed);
int16_t MOTOR_SendPositionCommandsToMotors(void);

#endif // MOTOR_H