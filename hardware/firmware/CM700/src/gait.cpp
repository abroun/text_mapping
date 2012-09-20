//------------------------------------------------------------------------------
#include "gait.h"
#include "time.h"
#include "motor.h"
#define NUM_ANGLES 16
#define KNEE_MOTOR_R 8
#define ANKLE_MOTOR_R 10
#define KNEE_MOTOR_L 7
#define ANKLE_MOTOR_L 9
#define MOTOR_SPEED 100
#define INTER_FRAME_DELAY_MS 1000

const uint16_t ANKLE_ANGLES[NUM_ANGLES] = {
	333,
	438,
	548,
	649,
	648,
	747,
	848,
	745,
	646,
	543,
	444,
	348,
	448,
	543,
	644,
	745,
};

const uint16_t KNEE_ANGLES[NUM_ANGLES] = {
    233,
    438,
    548,
    649,
    648,
    547,
    448,
    345,
    246,
    343,
    444,
    548,
    648,
    543,
    444,
    645,
};

static int16_t gCurrentSpeed = 0;
static uint32_t gLastFrameStartTimeMS = 0;
static int16_t gCurrentAngleIdx = 0;

//------------------------------------------------------------------------------
void GAIT_ProcessGait()
{

    if (gCurrentSpeed > 0)
    {
        uint32_t CurrentTimeMS = TIME_GetTimeMS();

        uint32_t TimeDiff = CurrentTimeMS - gLastFrameStartTimeMS;

        // Move on to current frame
        // NOTE: This throws away any time over INTER_FRAME_DELAY_MS so this may
        // need to be reworked for accurate animation playback
        if ( TimeDiff >= INTER_FRAME_DELAY_MS )
        {
            gCurrentAngleIdx = (gCurrentAngleIdx+1)%NUM_ANGLES;
            gLastFrameStartTimeMS = CurrentTimeMS;

            // Send frame to motors
            MOTOR_SetMotorPostionAndSpeed( ANKLE_MOTOR_R, 
                                           ANKLE_ANGLES[gCurrentAngleIdx], MOTOR_SPEED);
            MOTOR_SetMotorPostionAndSpeed( KNEE_MOTOR_R, 
                                           KNEE_ANGLES[gCurrentAngleIdx], MOTOR_SPEED);
	    MOTOR_SetMotorPostionAndSpeed( ANKLE_MOTOR_L, 
                                           ANKLE_ANGLES[gCurrentAngleIdx], MOTOR_SPEED);
	    MOTOR_SetMotorPostionAndSpeed( KNEE_MOTOR_L, 
                                           KNEE_ANGLES[gCurrentAngleIdx], MOTOR_SPEED);
        }

    }



}

//------------------------------------------------------------------------------
void GAIT_SetGaitSpeed( int16_t speed )
{
    gCurrentSpeed = speed;
}
