
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS( ${QT_DEFINITIONS} )

SET( MANUAL_MAP_BUILDER_SOURCES src/manual_map_builder.cpp src/mmb_main_window.cpp )
SET( MANUAL_MAP_BUILDER_HEADERS src/mmb_main_window.h )
SET( MANUAL_MAP_BUILDER_FORMS gui/mmb_main_window.ui)

QT4_WRAP_CPP( MANUAL_MAP_BUILDER_HEADERS_MOC ${MANUAL_MAP_BUILDER_HEADERS} )
QT4_WRAP_UI( MANUAL_MAP_BUILDER_FORMS_HEADERS ${MANUAL_MAP_BUILDER_FORMS} )

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

ADD_EXECUTABLE( manual_map_builder ${MANUAL_MAP_BUILDER_SOURCES} 
    ${MANUAL_MAP_BUILDER_HEADERS_MOC} 
    ${MANUAL_MAP_BUILDER_FORMS_HEADERS} )
TARGET_LINK_LIBRARIES( manual_map_builder ${QT_LIBRARIES} QVTK TextMapping ${BOOST_LIBRARIES} )
