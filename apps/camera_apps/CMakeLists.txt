
FIND_PACKAGE( OpenCV 2.3.1 REQUIRED )

INCLUDE_DIRECTORIES(
    ${OpenCV_INCLUDE_DIRS} )

ADD_EXECUTABLE( camera_calibration src/camera_calibration.cpp )
TARGET_LINK_LIBRARIES( camera_calibration ${OpenCV_LIBS} )

ADD_EXECUTABLE( stereo_calibration src/stereo_calibration.cpp )
TARGET_LINK_LIBRARIES( stereo_calibration ${OpenCV_LIBS} )

ADD_EXECUTABLE( chessboard_generator src/chessboard_generator.cpp )
TARGET_LINK_LIBRARIES( chessboard_generator ${OpenCV_LIBS} TextMapping )

