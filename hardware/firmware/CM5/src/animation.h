//------------------------------------------------------------------------------
// This module provides support for playing out animations on the robot.
// Animation frames are essentially sets of robot poses and animations are
// played out by playing frames in quick succession.
//------------------------------------------------------------------------------

#ifndef ANIMATION_H
#define ANIMATION_H

#include <stdint.h>

#define ANIM_MAX_NUM_FRAMES  6

void ANIM_Initialise();
void ANIM_ProcessAnimation();
void ANIM_SetAnimationFrame( int16_t frameIdx );
void ANIM_ClearCurrentFrame();
void ANIM_SetMotorPostionAndSpeed( uint16_t motorId, uint16_t position, uint16_t speed );
void ANIM_SetNextFrame( int16_t nextFrameIdx );
void ANIM_SetWaitTimeMS( uint16_t waitTimeMS );
void ANIM_StartAnimationFromFrame( int16_t frameIdx );
void ANIM_StopAnimation();
bool ANIM_LoadTableFromFlash();
void ANIM_SaveTableToFlash();

#endif // ANIMATION_H