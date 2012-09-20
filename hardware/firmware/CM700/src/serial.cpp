//------------------------------------------------------------------------------
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "CM700_Libs/include/serial.h"

#include "animation.h"
#include "gait.h"
#include "serial.h"

#define MAX_MSG_LENGTH 256

static char gSerialReadBuffer[ MAX_MSG_LENGTH ];
static char gMsgBuffer[ MAX_MSG_LENGTH ];
static int16_t gReadPos = 0;

static const char* SET_GAIT_SPEED_COMMAND = "SetGaitSpeed";
static const size_t SET_GAIT_SPEED_COMMAND_LENGTH = strlen( SET_GAIT_SPEED_COMMAND );
static const char* SET_ANIM_FRAME_COMMAND = "SetAnimFrame";
static const size_t SET_ANIM_FRAME_COMMAND_LENGTH = strlen( SET_ANIM_FRAME_COMMAND );
static const char* CLEAR_ANIM_FRAME_COMMAND = "ClearAnimFrame";
static const size_t CLEAR_ANIM_FRAME_COMMAND_LENGTH = strlen( CLEAR_ANIM_FRAME_COMMAND );
static const char* SET_MOTOR_COMMAND = "SetMotor";
static const size_t SET_MOTOR_COMMAND_LENGTH = strlen( SET_MOTOR_COMMAND );
static const char* SET_NEXT_ANIM_FRAME_COMMAND = "SetNextAnimFrame";
static const size_t SET_NEXT_ANIM_FRAME_COMMAND_LENGTH = strlen( SET_NEXT_ANIM_FRAME_COMMAND );
static const char* SET_ANIM_WAIT_TIME_MS_COMMAND = "SetAnimWaitTimeMS";
static const size_t SET_ANIM_WAIT_TIME_MS_COMMAND_LENGTH = strlen( SET_ANIM_WAIT_TIME_MS_COMMAND );
static const char* START_ANIM_FROM_FRAME_COMMAND = "StartAnimFromFrame";
static const size_t START_ANIM_FROM_FRAME_COMMAND_LENGTH = strlen( START_ANIM_FROM_FRAME_COMMAND );
static const char* STOP_ANIM_COMMAND = "StopAnim";
static const size_t STOP_ANIM_COMMAND_LENGTH = strlen( STOP_ANIM_COMMAND );
static const char* LOAD_ANIM_TABLE_COMMAND = "LoadAnimTable";
static const size_t LOAD_ANIM_TABLE_COMMAND_LENGTH = strlen( LOAD_ANIM_TABLE_COMMAND );
static const char* SAVE_ANIM_TABLE_COMMAND = "SaveAnimTable";
static const size_t SAVE_ANIM_TABLE_COMMAND_LENGTH = strlen( SAVE_ANIM_TABLE_COMMAND );

//------------------------------------------------------------------------------
void SERIAL_ProcessSerialComms()
{
    // Read in data from the serial port
    int16_t spaceAvailable = MAX_MSG_LENGTH - gReadPos;
    int16_t numExtraBytes = serial_read( (unsigned char*)&gSerialReadBuffer[ gReadPos ], spaceAvailable );
    
    // Check to see if there is a message in the buffer
    int16_t msgEndIdx = -1;
    for ( int16_t i = gReadPos; i < gReadPos + numExtraBytes; i++ )
    {
        if ( '\n' == gSerialReadBuffer[ i ] )
        {
            // We've found a message
            msgEndIdx = i;
            break;
        }
    }

    if ( msgEndIdx >= 0 )
    {
        // We have a message, copy it out
        memcpy( gMsgBuffer, gSerialReadBuffer, msgEndIdx );
        gMsgBuffer[ msgEndIdx ] = '\0'; // NUL terminate the string
    
        int16_t numBytesLeft = gReadPos + numExtraBytes - (msgEndIdx+1);
        if ( numBytesLeft > 0 )
        {
            // Move the stuff left in the serial read buffer back to the start
            memcpy( gSerialReadBuffer, &gSerialReadBuffer[ msgEndIdx + 1 ], numBytesLeft );
        }
        gReadPos = numBytesLeft;
    
        // Parse and process the message    
        bool bMsgParsed = false;
        
        if ( strncasecmp( gMsgBuffer, SET_GAIT_SPEED_COMMAND, SET_GAIT_SPEED_COMMAND_LENGTH ) == 0 )
        {
            int16_t gaitSpeed;
            if ( sscanf( &gMsgBuffer[ SET_GAIT_SPEED_COMMAND_LENGTH ], "%i", &gaitSpeed ) == 1 )
            {
                //printf( "Setting gait speed to %i\n", gaitSpeed );
                GAIT_SetGaitSpeed( gaitSpeed );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, SET_ANIM_FRAME_COMMAND, SET_ANIM_FRAME_COMMAND_LENGTH ) == 0 )
        {
            int16_t animFrame;
            if ( sscanf( &gMsgBuffer[ SET_ANIM_FRAME_COMMAND_LENGTH ], "%i", &animFrame ) == 1 )
            {
                ANIM_SetAnimationFrame( animFrame );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, CLEAR_ANIM_FRAME_COMMAND, CLEAR_ANIM_FRAME_COMMAND_LENGTH ) == 0 )
        {
            ANIM_ClearCurrentFrame();
            bMsgParsed = true;
        }
        else if ( strncasecmp( gMsgBuffer, SET_MOTOR_COMMAND, SET_MOTOR_COMMAND_LENGTH ) == 0 )
        {
            int16_t motorId = 0;
            int16_t position = 0;
            int16_t speed = 0;
            if ( sscanf( &gMsgBuffer[ SET_MOTOR_COMMAND_LENGTH ], 
                "%i %i %i", &motorId, &position, &speed ) >= 2 )    // Only require the first 2 arguments, speed can default to 0
            {
                ANIM_SetMotorPostionAndSpeed( motorId, position, speed );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, SET_NEXT_ANIM_FRAME_COMMAND, SET_NEXT_ANIM_FRAME_COMMAND_LENGTH ) == 0 )
        {
            int16_t nextAnimFrame;
            if ( sscanf( &gMsgBuffer[ SET_NEXT_ANIM_FRAME_COMMAND_LENGTH ], "%i", &nextAnimFrame ) == 1 )
            {
                ANIM_SetNextFrame( nextAnimFrame );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, SET_ANIM_WAIT_TIME_MS_COMMAND, SET_ANIM_WAIT_TIME_MS_COMMAND_LENGTH ) == 0 )
        {
            uint16_t waitTimeMS;
            if ( sscanf( &gMsgBuffer[ SET_ANIM_WAIT_TIME_MS_COMMAND_LENGTH ], "%u", &waitTimeMS ) == 1 )
            {
                ANIM_SetWaitTimeMS( waitTimeMS );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, START_ANIM_FROM_FRAME_COMMAND, START_ANIM_FROM_FRAME_COMMAND_LENGTH ) == 0 )
        {
            int16_t animFrame;
            if ( sscanf( &gMsgBuffer[ START_ANIM_FROM_FRAME_COMMAND_LENGTH ], "%i", &animFrame ) == 1 )
            {
                ANIM_StartAnimationFromFrame( animFrame );
                bMsgParsed = true;
            }
        }
        else if ( strncasecmp( gMsgBuffer, STOP_ANIM_COMMAND, STOP_ANIM_COMMAND_LENGTH ) == 0 )
        {
            ANIM_StopAnimation();
            bMsgParsed = true;
        }
        else if ( strncasecmp( gMsgBuffer, LOAD_ANIM_TABLE_COMMAND, LOAD_ANIM_TABLE_COMMAND_LENGTH ) == 0 )
        {
            ANIM_LoadTableFromFlash();
            bMsgParsed = true;
        }
        else if ( strncasecmp( gMsgBuffer, SAVE_ANIM_TABLE_COMMAND, SAVE_ANIM_TABLE_COMMAND_LENGTH ) == 0 )
        {
            ANIM_SaveTableToFlash();
            bMsgParsed = true;
        }
        
        if ( !bMsgParsed )
        {
            printf( "Error: Can't parse message\n" );
        }
    }
    else
    {
        // No message was found but have we now run out of space
        gReadPos = gReadPos + numExtraBytes;
        if ( gReadPos >= MAX_MSG_LENGTH )
        {
            gReadPos = 0;   // Run out of space so clear the buffer and hope for the best
        }
    } 
}
