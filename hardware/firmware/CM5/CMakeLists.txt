CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )
PROJECT( CM5_Firmware )

# Point to the avr compiler
SET( CMAKE_C_COMPILER avr-gcc )
SET( CMAKE_CXX_COMPILER avr-g++ )

# Set the microcontroller type
set( CMAKE_C_FLAGS "-mmcu=atmega128 -DF_CPU=16000000UL -Os" CACHE STRING "" FORCE )
set( CMAKE_CXX_FLAGS "-mmcu=atmega128 -DF_CPU=16000000UL -Os" CACHE STRING "" FORCE )

INCLUDE_DIRECTORIES(
    .
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../../3rdParty )

LINK_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/../../3rdParty/CM5_Libs/lib )

SET( srcFiles
    src/main.cpp
    src/animation.cpp
    src/gait.cpp
    src/motor.cpp
    src/serial.cpp
    src/time.cpp )

ADD_EXECUTABLE( firmware ${srcFiles} )
TARGET_LINK_LIBRARIES( firmware dynamixel serial )

ADD_CUSTOM_COMMAND( TARGET firmware
                    POST_BUILD
                    COMMAND avr-objcopy -j .text -j .data -O ihex firmware firmware.hex )

