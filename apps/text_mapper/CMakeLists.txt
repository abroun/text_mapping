
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS( ${QT_DEFINITIONS} )

FIND_PACKAGE( OpenCV 2.3.1 REQUIRED )

SET( TEXT_MAPPER_SOURCES 
    src/camera.cpp
    src/frame_dialog.cpp
	src/image_view_dialog.cpp
	src/text_mapper.cpp 
	src/tm_main_window.cpp )
SET( TEXT_MAPPER_HEADERS
	src/frame_dialog.h
	src/image_view_dialog.h
	src/tm_main_window.h )
SET( TEXT_MAPPER_FORMS 
	gui/frame_dialog.ui
	gui/image_view_dialog.ui
	gui/tm_main_window.ui )

QT4_WRAP_CPP( TEXT_MAPPER_HEADERS_MOC ${TEXT_MAPPER_HEADERS} )
QT4_WRAP_UI( TEXT_MAPPER_FORMS_HEADERS ${TEXT_MAPPER_FORMS} )

INCLUDE_DIRECTORIES( 
	${CMAKE_CURRENT_BINARY_DIR} 
	${OpenCV_INCLUDE_DIRS} )

ADD_EXECUTABLE( text_mapper ${TEXT_MAPPER_SOURCES} 
    ${TEXT_MAPPER_HEADERS_MOC} 
    ${TEXT_MAPPER_FORMS_HEADERS} )
TARGET_LINK_LIBRARIES( text_mapper ${QT_LIBRARIES} QVTK ${OpenCV_LIBS} TextMapping ${BOOST_LIBRARIES} )
