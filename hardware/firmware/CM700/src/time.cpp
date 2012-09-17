//------------------------------------------------------------------------------
// This module works under the assumption that CLK_io (which drives the timer 
// modules) is the same as CLK_cpu which is defined in CMakeLists.txt and in 
// the Robotis samples as 16000000 (16MHz). If this assumption is incorrect
// then the timing will be inaccurate.
//------------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>
#include "time.h"

//------------------------------------------------------------------------------
// Assuming that CLK_io is 16000000 we divide it by 256 (see setting for TCCR1B)
// which gives 62500 for 1 second. We then set the timer so that it will 
// overflow after 125 counts letting us measure a time of 0.002 seconds (2 ms)
#define RELOAD_VALUE (0xFFFF - (125 - 1))

// This will overflow if the firmware runs for more than 49.71 days
static uint32_t gTimeMS = 0;

//------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect) 
{
    TCNT1 = RELOAD_VALUE;
    gTimeMS += 2;
}

//------------------------------------------------------------------------------
void TIME_Initialise()
{
    TIMSK1 = 0x01; // enabled global and timer overflow interrupt;
    TCCR1A = 0x00; // normal operation page 148 (mode0);
    TCNT1 = RELOAD_VALUE; 
    TCCR1B = 0x04; // start timer and set clock division to 256
}

//------------------------------------------------------------------------------
float TIME_GetTime()
{
    // Return the time in seconds
    return ((float)gTimeMS)/1000.0;
}

//------------------------------------------------------------------------------
uint32_t TIME_GetTimeMS()
{
    return gTimeMS;
}