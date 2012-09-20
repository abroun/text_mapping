#! /usr/bin/python

# Test script to show how to send position commands to servos attached to a CM-5 or CM-700

import serial
import optparse
import time
import sys

DEFAULT_MOTOR_SPEED = 100

#---------------------------------------------------------------------------------------------------
if sys.platform == "win32":
	DEFAULT_SERIAL_PORT = "COM3"
else:
	DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD_RATE = 57600

parser = optparse.OptionParser( usage="usage: %prog [options]" )
parser.add_option( "-p", "--port", dest="port", default=DEFAULT_SERIAL_PORT,
                  help="The serial port to try to connect on" )
parser.add_option("-b", "--baud", dest="baudRate", default=DEFAULT_BAUD_RATE,
                  help="Baud rate to use for communication" )

(options, args) = parser.parse_args()

# Open serial port to communicate with the control board
serialChannel = serial.Serial( options.port, int( options.baudRate ) )

# Stop any existing animation
serialChannel.write( "StopAnim\n" )
time.sleep( 0.5 )

# Set up a dummy animation which will allow us to send position commands without delay
serialChannel.write( "SetAnimFrame 0\n" )
serialChannel.write( "ClearAnimFrame\n" )
serialChannel.write( "SetNextAnimFrame 0\n" )
serialChannel.write( "SetAnimWaitTimeMS 0\n" )
serialChannel.write( "StartAnimFromFrame 0\n" )

# Now send position commands in a loop
while True:
    
    time.sleep( 2.0 )
    serialChannel.write( "SetMotor 16 0 200\n" )
    serialChannel.write( "SetMotor 18 0 200\n" )
    
    time.sleep( 2.0 )
    serialChannel.write( "SetMotor 16 1023 200\n" )
    serialChannel.write( "SetMotor 18 1023 200\n" )


