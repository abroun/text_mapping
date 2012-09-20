#! /usr/bin/python

# This program takes a firmware file in the Intel hex format and uses it
# to flash a Robotis control board.

import serial
import optparse
import time
import re
import sys

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD_RATE = 57600

parser = optparse.OptionParser( usage="usage: %prog [options] firmwareHexFile" )
parser.add_option( "-p", "--port", dest="port", default=DEFAULT_SERIAL_PORT,
                  help="The serial port to try to connect on" )
parser.add_option("-b", "--baud", dest="baudRate", default=DEFAULT_BAUD_RATE,
                  help="Baud rate to use for communication" )

(options, args) = parser.parse_args()

# Open up the hex file
if len( args ) <= 0:
    print "Error: No firmware file provided"
    sys.exit( -1 )

hexFile = open( args[ 0 ], "r" )

# Open serial port to communicate with the control board
serialChannel = serial.Serial( options.port, int( options.baudRate ) )

targetBoard = None
bootLoaderVersion = "Unknown"
print "Please switch the device off and on start the connection"

# Send the reset character until we get a reply from the bootloader
replyText = ""
testRegEx = re.compile("SYSTEM O\.K\. \((.*) Boot loader V([0-9\.]+)\)")

while True:
    serialChannel.write( "#" )
    if serialChannel.inWaiting() > 0:
        replyText += serialChannel.read( serialChannel.inWaiting() )
        
        # Check to see if we've heard from the boot loader
        match = testRegEx.search( replyText )
        if match != None:
            groups = match.groups()
            targetBoard = groups[ 0 ]
            bootLoaderVersion = groups[ 1 ]
            break
    
    time.sleep( 0.05 )  # Try to avoid major spam
    
if targetBoard == None:
    print "Error: Unable to access boot loader"
    sys.exit( -1 )
    
print "Connected to", targetBoard, "with bootloader version", bootLoaderVersion

# Ready the bootloader to receive the firmware
serialChannel.write( "\nld\n" )
time.sleep( 0.2 )
while serialChannel.inWaiting():
    serialChannel.read( serialChannel.inWaiting() )
    
# Now send the firmware
print "Sending firmware"
checkSum = 0
for line in hexFile:
    asciiLine = line.lstrip( ":" ).rstrip()
    if len( asciiLine )%2 != 0:
        print "Error: Encountered a line in the hex file without a whole number of bytes"
        sys.exit( -1 )
    
    binaryLine = ""
    for byteStartIdx in range( 8, len( asciiLine ) - 2, 2 ):
        byteValue = int( asciiLine[ byteStartIdx:byteStartIdx+2 ], 16 )
        checkSum += byteValue
        binaryLine += chr( byteValue )
    
    #print binaryLine
    serialChannel.write( binaryLine )
    if serialChannel.inWaiting() > 0:
        print serialChannel.read( serialChannel.inWaiting() )

serialChannel.write( chr( checkSum & 0xFF ) )    # Send checksum and we've finished


print "Firmware sent"
        
# Wait to see if we succeeded
WAIT_TIME = 0.5
replyText = ""
startTime = time.time()
transferFailed = True

while True:
    if serialChannel.inWaiting() > 0:
        replyText += serialChannel.read( serialChannel.inWaiting() )
        if replyText.find( "Success" ) >= 0:
            transferFailed = False
            break
        
    if time.time() - startTime > WAIT_TIME:
        break

if transferFailed:
    print "Error: Transfer failed"
    print "Returned text was", replyText
    sys.exit( -1 )
    
print "Starting the program"
serialChannel.write( "go\n" )