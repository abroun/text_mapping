
FIND_PACKAGE( OpenCV 2.3.1 REQUIRED )

INCLUDE_DIRECTORIES(
    ${OpenCV_INCLUDE_DIRS} )

SET( SRC_FILES src/camera_calibration.cpp )

ADD_EXECUTABLE( camera_calibration ${SRC_FILES} )
TARGET_LINK_LIBRARIES( camera_calibration TextMapping ${OpenCV_LIBS} )
