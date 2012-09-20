//------------------------------------------------------------------------------

#include <string.h>

#include "motor.h"
#include "CM700_Libs/include/serial.h"
#include "CM700_Libs/include/dynamixel.h"
#include "CM700_Libs/src/Dynamixel/dxl_hal.h"


/// Control table address
#define P_GOAL_POSITION_L   30
#define P_GOAL_POSITION_H   31
#define P_GOAL_SPEED_L      32
#define P_GOAL_SPEED_H      33


struct joint_parameters {
  uint16_t pos;
  uint16_t speed;
  bool bPositionSet;    // Set the first time a position is sent to the motor
};

static struct joint_parameters joint[MOTOR_NJOINTS];

//------------------------------------------------------------------------------
void MOTOR_ConfigureAllMotors()
{
    memset( joint, 0, sizeof( joint ) );
}


int16_t MOTOR_SetMotorPostionAndSpeed(uint16_t id, uint16_t pos, uint16_t speed ) {
  if(pos > 1023)
    pos = 1023;
  
  if(speed > 1023)
    speed = 1023;
  
  if(id >= MOTOR_NJOINTS)
    return MOTOR_ERROR;
  
  // Set goal speed
  joint[id].pos = pos;
  joint[id].speed = speed;
  joint[id].bPositionSet = true;
  
  return MOTOR_SUCCESS;
}

//------------------------------------------------------------------------------
// 
//  Data Format for SYNC write:
//  			ID		0xFE
//			Length		((L+1)*N )+4  	L: Data Length per RX-28, N: Number of motors
//			Instruction	0x83
//			Start address to write data	parameter 1
//			Length of data to write		parameter 2 = Number of motors
//			
//			First ID of RX-28		parameter 3
//			First data of the 1st RX-28
//			..
//			Lth data of the 1st RX-28 
//
//     #define ID				(2)
//     #define LENGTH				(3)
//     #define INSTRUCTION			(4)
//     #define ERRBIT				(4)
//     #define PARAMETER			(5)
//     #define DEFAULT_BAUDNUMBER		(1)
//
int16_t MOTOR_SendPositionCommandsToMotors(void)
{
  int16_t i;
  int16_t numJointsSent = 0;
  
  dxl_set_txpacket_id(BROADCAST_ID);
  dxl_set_txpacket_instruction(INST_SYNC_WRITE);
  dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
  dxl_set_txpacket_parameter(1, 4); // Write 4 bytes
  
  for( i=0; i<MOTOR_NJOINTS; i++ )
  {
      if ( !joint[i].bPositionSet )
      {
          // Don't send a joint value if we haven't got a command
          continue;
      }
      
	  dxl_set_txpacket_parameter(2+(5*numJointsSent), i);
	  dxl_set_txpacket_parameter(2+(5*numJointsSent)+1, dxl_get_lowbyte(joint[i].pos));
	  dxl_set_txpacket_parameter(2+(5*numJointsSent)+2, dxl_get_highbyte(joint[i].pos));
	  dxl_set_txpacket_parameter(2+(5*numJointsSent)+3, dxl_get_lowbyte(joint[i].speed));
	  dxl_set_txpacket_parameter(2+(5*numJointsSent)+4, dxl_get_highbyte(joint[i].speed));
      
      numJointsSent++;
  }
  
  dxl_set_txpacket_length((5*numJointsSent)+4);
  
  dxl_txrx_packet();
  return MOTOR_SUCCESS;
}