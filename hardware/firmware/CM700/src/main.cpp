//------------------------------------------------------------------------------
// This file contains the main function for the BRL firmware. It's fairly 
// simplistic consisting of an initial initialisation step followed by a loop
// which updates various periodic processes and then sends the current
// position commands down to the motors
//------------------------------------------------------------------------------

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "CM700_Libs/include/serial.h"
#include "CM700_Libs/include/dynamixel.h"

#include "animation.h"
#include "gait.h"
#include "motor.h"
#include "serial.h"
#include "time.h"

// Default setting
#define DEFAULT_BAUDNUM     1 // 1Mbps
#define DEFAULT_ID          1

//------------------------------------------------------------------------------
void SetupAnimationTest();

//------------------------------------------------------------------------------
int16_t main()
{   
    // Initialisation step
    TIME_Initialise();
    
    serial_initialize( 57600 );
    dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
    sei();    // Interrupt Enable   

    printf( "\n\nBRL Firmware\n\n" );
    
    ANIM_Initialise();
    MOTOR_ConfigureAllMotors();        // Sets default parameters on the RX-28s
    
    // Setup optional animation test
    //SetupAnimationTest();
    
    // Now start the main loop
    while ( 1 )
    {
        SERIAL_ProcessSerialComms();
        ANIM_ProcessAnimation();
        GAIT_ProcessGait();
        
        MOTOR_SendPositionCommandsToMotors();
    }

    return 0;
}

//------------------------------------------------------------------------------
// This routine is intended to provide a simple example of how to use the
// animation system. It sets up two frames which have motor positions for
// motors 1 and 2. The frames are linked to each other to form a looping 
// animation and a delay of 1 second (1000ms) is set between each frame.
//------------------------------------------------------------------------------
void SetupAnimationTest()
{
    ANIM_SetAnimationFrame( 0 );
    ANIM_ClearCurrentFrame();
    ANIM_SetMotorPostionAndSpeed( 1, 0, 0 );
    ANIM_SetMotorPostionAndSpeed( 2, 1023, 0 );
    ANIM_SetNextFrame( 1 );
    ANIM_SetWaitTimeMS( 1000 );
    
    ANIM_SetAnimationFrame( 1 );
    ANIM_ClearCurrentFrame();
    ANIM_SetMotorPostionAndSpeed( 1, 1023, 0 );
    ANIM_SetMotorPostionAndSpeed( 2, 0, 0 );
    ANIM_SetNextFrame( 0 );
    ANIM_SetWaitTimeMS( 1000 );
    
    ANIM_StartAnimationFromFrame( 0 );
}