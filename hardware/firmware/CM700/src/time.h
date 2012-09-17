//------------------------------------------------------------------------------
// This module provides a way of tracking time in the firmware. It uses
// timer 1 of the Atmega2561 to keep track of the amount of time that has
// passed since TIME_Initialise was first called.
//------------------------------------------------------------------------------

#ifndef TIME_H
#define TIME_H

#include <stdint.h>

void TIME_Initialise();

// Returns time in seconds
float TIME_GetTime();

// Returns time in milliseconds
uint32_t TIME_GetTimeMS();

#endif // TIME_H