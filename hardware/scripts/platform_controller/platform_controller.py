#! /usr/bin/python

import sys
import math
import os.path
import time

import pygtk
pygtk.require('2.0')
import gtk
import gobject

import serial
import optparse

#---------------------------------------------------------------------------------------------------
class MainWindow:

    BASE_MOTOR_ID = 18
    LEFT_MOTOR_ID = 16
    RIGHT_MOTOR_ID = 14
    
    MOTOR_SPEED = 200
    MAX_MOTOR_POS = 1023
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialChannel ):
    
        self.serialChannel = serialChannel
        self.scriptPath = os.path.dirname( __file__ )
        self.lastPositionCommandDict = None
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/gui/platform_controller.glade" )            
        
        self.window = builder.get_object( "winMain" )  
        self.checkSendAngles = builder.get_object( "checkSendAngles" )
        self.adjAxisAngleX = builder.get_object( "adjAxisAngleX" )
        self.adjAxisAngleX.set_value( int( self.MAX_MOTOR_POS/2.0 ) )
        self.adjAxisAngleY = builder.get_object( "adjAxisAngleY" )
        self.adjAxisAngleY.set_value( int( self.MAX_MOTOR_POS/2.0 ) )
        
        builder.connect_signals( self )
               
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.gdk.threads_init()
        gtk.main()
    
    #---------------------------------------------------------------------------
    def onBtnZeroAxesClicked( self, widget ):
        
        self.adjAxisAngleX.set_value( int( self.MAX_MOTOR_POS/2.0 ) )
        self.adjAxisAngleY.set_value( int( self.MAX_MOTOR_POS/2.0 ) )

    #---------------------------------------------------------------------------
    def update( self ):

        UPDATE_FREQUENCY = 30       # Hz
        IDEAL_TIME_DIFF = 1.0/float( UPDATE_FREQUENCY )
        lastTime = time.time()

        while 1:
            
            curTime = time.time()
            timeDiff = curTime - lastTime
            
            if timeDiff > IDEAL_TIME_DIFF:
                
                if self.checkSendAngles.get_active():
                    
                    positionCommandDict = {}
                    
                    # Axis X
                    positionCommandDict[ self.LEFT_MOTOR_ID ] = int( self.adjAxisAngleX.get_value() )
                    positionCommandDict[ self.RIGHT_MOTOR_ID ] = int( self.MAX_MOTOR_POS - self.adjAxisAngleX.get_value() )
                    
                    # Axis Y
                    positionCommandDict[ self.BASE_MOTOR_ID ] = int( self.adjAxisAngleY.get_value() )
                    
                    # Only send the angles if they're different in some way from
                    # last lot we sent
                    if positionCommandDict != self.lastPositionCommandDict:
                        
                        for motorIdx in positionCommandDict:
                        
                            pos = positionCommandDict[ motorIdx ]
                            
                            # Constrain the position to a valid motor position
                            pos = int( max( 0, min( pos, self.MAX_MOTOR_POS ) ) )   
                            
                            serialChannel.write( "SetMotor {0} {1} {2}\n".format( motorIdx, pos, self.MOTOR_SPEED ) )
                
                lastTime = curTime
            else:
                time.sleep( IDEAL_TIME_DIFF - timeDiff )
                
            yield True
            
        yield False

#-------------------------------------------------------------------------------
if __name__ == "__main__":

    if sys.platform == "win32":
        DEFAULT_SERIAL_PORT = "COM6"
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
    
    mainWindow = MainWindow( serialChannel )
    mainWindow.main()
