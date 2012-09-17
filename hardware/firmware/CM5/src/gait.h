//------------------------------------------------------------------------------
// This module generates the gait of the robot so that it can walk around
//------------------------------------------------------------------------------

#ifndef GAIT_H
#define GAIT_H

#include <stdint.h>

void GAIT_ProcessGait();
void GAIT_SetGaitSpeed( int16_t speed );

#endif // GAIT_H