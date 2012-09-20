//------------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "animation.h"
#include "motor.h"
#include "time.h"

//------------------------------------------------------------------------------
// NOTE: This should be changed whenever the MotorState or AnimationFrame
// structs change.
#define DATA_VERSION    2   
#define EEPROM_TABLE_START_ADDRESS  0x0000

struct MotorState
{
    uint16_t mPosition;
    uint16_t mSpeed;
    bool mbPositionSet;
};

struct AnimationFrame
{
    MotorState mMotorStates[ MOTOR_NJOINTS ];
    int16_t mNextFrameIdx;
    uint16_t mWaitTimeMS;   // Time until the next frame in milliseconds

    void clear()
    {
        memset( mMotorStates, 0, sizeof( mMotorStates ) );
    }
};

//------------------------------------------------------------------------------
// Module variables
static int16_t gEditFrameIdx = 0;
static int16_t gCurPlayingFrameIdx = -1;
static uint32_t gFrameStartTimeMS = 0;

static AnimationFrame gAnimFrames[ ANIM_MAX_NUM_FRAMES ];

//------------------------------------------------------------------------------
// Function declarations
static void SendFrameToMotors( int16_t frameIdx );
//static uint8_t ReadByteFromEEPROM( uint16_t address );
//static void WriteByteToEEPROM( uint16_t address, uint8_t data );

//------------------------------------------------------------------------------
void ANIM_Initialise()
{
    // Try to first load the animation table from flash
    if ( !ANIM_LoadTableFromFlash() )
    {
        // Clear out the animation table
        for ( int16_t i = 0; i < ANIM_MAX_NUM_FRAMES; i++ )
        {
            gAnimFrames[ i ].clear();
        }
    }
}

//------------------------------------------------------------------------------
void ANIM_ProcessAnimation()
{
    if ( gCurPlayingFrameIdx >= 0 && gCurPlayingFrameIdx < ANIM_MAX_NUM_FRAMES
        && gAnimFrames[ gCurPlayingFrameIdx ].mNextFrameIdx >= 0 )
    {
        // Update the animation
        uint32_t curTimeMS = TIME_GetTimeMS();
        
        uint32_t timeSinceStartOfFrameMS = curTimeMS - gFrameStartTimeMS;
        uint32_t frameWaitTimeMS = gAnimFrames[ gCurPlayingFrameIdx ].mWaitTimeMS;
        if ( timeSinceStartOfFrameMS >= frameWaitTimeMS )
        {
            // We need to move onto the next frame
            gCurPlayingFrameIdx = gAnimFrames[ gCurPlayingFrameIdx ].mNextFrameIdx;
            SendFrameToMotors( gCurPlayingFrameIdx );
            gFrameStartTimeMS = curTimeMS;
        }
    }
}

//------------------------------------------------------------------------------
void ANIM_SetAnimationFrame( int16_t frameIdx )
{
    if ( frameIdx >= 0 && frameIdx < ANIM_MAX_NUM_FRAMES )
    {
        gEditFrameIdx = frameIdx;
    }
}

//------------------------------------------------------------------------------
void ANIM_ClearCurrentFrame()
{
    gAnimFrames[ gEditFrameIdx ].clear();
}

//------------------------------------------------------------------------------
void ANIM_SetMotorPostionAndSpeed( uint16_t motorId, uint16_t position, uint16_t speed )
{
    if ( motorId < MOTOR_NJOINTS )
    {        
        gAnimFrames[ gEditFrameIdx ].mMotorStates[ motorId ].mPosition = position;
        gAnimFrames[ gEditFrameIdx ].mMotorStates[ motorId ].mSpeed = speed;
        gAnimFrames[ gEditFrameIdx ].mMotorStates[ motorId ].mbPositionSet = true;
    }
}

//------------------------------------------------------------------------------
void ANIM_SetNextFrame( int16_t nextFrameIdx )
{
    if ( nextFrameIdx < -1 || nextFrameIdx >= ANIM_MAX_NUM_FRAMES )
    {
        nextFrameIdx = -1;
    }
    
    gAnimFrames[ gEditFrameIdx ].mNextFrameIdx = nextFrameIdx;
}

//------------------------------------------------------------------------------
void ANIM_SetWaitTimeMS( uint16_t waitTimeMS )
{
    gAnimFrames[ gEditFrameIdx ].mWaitTimeMS = waitTimeMS;
}

//------------------------------------------------------------------------------
void ANIM_StartAnimationFromFrame( int16_t frameIdx )
{
    if ( frameIdx >= 0 && frameIdx < ANIM_MAX_NUM_FRAMES )      
    {
        // Start playing the animation
        gCurPlayingFrameIdx = frameIdx;
        SendFrameToMotors( gCurPlayingFrameIdx );
        gFrameStartTimeMS = TIME_GetTimeMS();
    }
}

//------------------------------------------------------------------------------
void ANIM_StopAnimation()
{
    // Invalidate the current frame index to stop the animations
    gCurPlayingFrameIdx = -1;
}

//------------------------------------------------------------------------------
// The animation table is stored in EEPROM memory in the following format
//
//  DATA_VERSION    (uint16_t)
//  gAnimFrames
//  CHECKSUM        (uint32_t)  : Calculated by adding all bytes (including data version) together
//
bool ANIM_LoadTableFromFlash()
{
    printf( "Trying to load Animation Table from flash\n" );
    cli();  // Disable interrupts
    
    bool bTableLoaded = false;
    
    uint8_t* byteAddress = (uint8_t*)(EEPROM_TABLE_START_ADDRESS);
    uint16_t dataVersion = ((uint16_t)eeprom_read_byte( byteAddress++ ) << 8) 
        | (uint16_t)eeprom_read_byte( byteAddress++ );
    if ( DATA_VERSION != dataVersion )
    {
        sei();  // Enable interrupts
        printf( "Got data version of %i when expecting %i. Ignoring stored data\n", 
                dataVersion, DATA_VERSION );
    }
    else
    {
        // Start the checksum
        uint32_t calculatedChecksum = 0;
        calculatedChecksum += dataVersion >> 8;
        calculatedChecksum += dataVersion & 0xFF;
        
        // Read in the data
        uint16_t numDataBytesToRead = sizeof( gAnimFrames );
        uint8_t* pDstBytes = (uint8_t*)gAnimFrames;
        for ( uint16_t byteIdx = 0; byteIdx < numDataBytesToRead; byteIdx++ )
        {
            uint8_t byte = eeprom_read_byte( byteAddress++ );
            calculatedChecksum += byte;
            pDstBytes[ byteIdx ] = byte;
        }
        
        // Now read in the checksum
        uint32_t storedChecksum = ((uint32_t)eeprom_read_byte( byteAddress++ ) << 24) 
            | ((uint32_t)eeprom_read_byte( byteAddress++ ) << 16) 
            | ((uint32_t)eeprom_read_byte( byteAddress++ ) << 8) 
            | eeprom_read_byte( byteAddress++ );
        
        sei();  // Enable interrupts
            
        // Compare stored to calculated checksum
        if ( storedChecksum != calculatedChecksum )
        {
            printf( "Error: The stored checksum is different from the calculated checksum. The data may be corrupt\n" );
        }
        else
        {
            printf( "Data loaded\n" );
            bTableLoaded = true;
        }
    }
    
    return bTableLoaded;
}

//------------------------------------------------------------------------------
void ANIM_SaveTableToFlash()
{
    uint16_t dataVersion = DATA_VERSION;
    
    printf( "Saving Animation Table to flash\n" );
    cli();  // Disable interrupts
    
    // Wait for any write to finish and then put EECR in a known state
    while ( EECR & (1<<EEPE) )
        ;
    EECR = 0;
    
    // Start the checksum
    uint32_t calculatedChecksum = 0;
    calculatedChecksum += dataVersion >> 8;
    calculatedChecksum += dataVersion & 0xFF;
    
    // Store the data version
    uint8_t* byteAddress = (uint8_t*)(EEPROM_TABLE_START_ADDRESS);
    eeprom_write_byte( byteAddress++, dataVersion >> 8 );
    eeprom_write_byte( byteAddress++, dataVersion & 0xFF );

    // Store the animation data
    uint16_t numDataBytesToStore = sizeof( gAnimFrames );
    uint8_t* pSrcBytes = (uint8_t*)gAnimFrames;
    for ( uint16_t byteIdx = 0; byteIdx < numDataBytesToStore; byteIdx++ )
    {
        uint8_t byte = pSrcBytes[ byteIdx ];
        calculatedChecksum += byte;
        eeprom_write_byte( byteAddress++, byte );
    }
    
    // Store the checksum
    eeprom_write_byte( byteAddress++, calculatedChecksum >> 24 );
    eeprom_write_byte( byteAddress++, (calculatedChecksum >> 16) & 0xFF );
    eeprom_write_byte( byteAddress++, (calculatedChecksum >> 8) & 0xFF );
    eeprom_write_byte( byteAddress++, calculatedChecksum & 0xFF );
    
    // Finished
    sei();  // Enable interrupts
    printf( "Done\n" );
}

//------------------------------------------------------------------------------
static void SendFrameToMotors( int16_t frameIdx )
{
    if ( frameIdx >= 0 && frameIdx < ANIM_MAX_NUM_FRAMES )
    {
        for ( int16_t motorIdx = 0; motorIdx < MOTOR_NJOINTS; motorIdx++ )
        {
            MotorState& motorState = gAnimFrames[ frameIdx ].mMotorStates[ motorIdx ];
            
            // Only send the motor state if it's been explicitly set
            if ( motorState.mbPositionSet )
            {
                MOTOR_SetMotorPostionAndSpeed( motorIdx, motorState.mPosition, motorState.mSpeed );
            }
        }
    }
}
