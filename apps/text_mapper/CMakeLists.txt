
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS( ${QT_DEFINITIONS} )

FIND_PACKAGE( OpenCV 2.3.1 REQUIRED )

SET( TEXT_MAPPER_SOURCES 
    src/camera.cpp
    src/frame_data.cpp
    src/frame_dialog.cpp
	src/image_view_dialog.cpp
	src/key_point.cpp
	src/text_mapper.cpp 
	src/tm_main_window.cpp 
	src/text_detection.cpp
	src/dialogs/model_view_dialog.cpp
	src/vtk/vtk_key_point_instances_source.cpp )
SET( TEXT_MAPPER_HEADERS
	src/frame_dialog.h
	src/image_view_dialog.h
	src/dialogs/model_view_dialog.h
	src/tm_main_window.h )
SET( TEXT_MAPPER_FORMS 
	gui/frame_dialog.ui
	gui/image_view_dialog.ui
	gui/model_view_dialog.ui
	gui/tm_main_window.ui )

QT4_WRAP_CPP( TEXT_MAPPER_HEADERS_MOC ${TEXT_MAPPER_HEADERS} )
QT4_WRAP_UI( TEXT_MAPPER_FORMS_HEADERS ${TEXT_MAPPER_FORMS} )

INCLUDE_DIRECTORIES( 
	${CMAKE_CURRENT_BINARY_DIR} 
	${OpenCV_INCLUDE_DIRS} 
	"C:/Program Files (x86)/Tesseract-OCR/Build/api"
	"C:/Program Files (x86)/Tesseract-OCR/Build/ccstruct"
	"C:/Program Files (x86)/Tesseract-OCR/Build/ccmain"
	"C:/Program Files (x86)/Tesseract-OCR/Build/ccutil"
	"C:/Program Files (x86)/Tesseract-OCR/Build/cube"
	"C:/Program Files (x86)/Tesseract-OCR/Build/classify"
	"C:/Program Files (x86)/Tesseract-OCR/Build/cutil"
	"C:/Program Files (x86)/Tesseract-OCR/Build/dict"
	"C:/Program Files (x86)/Tesseract-OCR/Build/image"
	"C:/Program Files (x86)/Tesseract-OCR/Build/neural_networks/runtime"
	"C:/Program Files (x86)/Tesseract-OCR/Build/textord")

IF( WIN32 )
LINK_DIRECTORIES(
	"C:/Program Files (x86)/Tesseract-OCR/Build/vs2010/Release.dll/")
ENDIF( WIN32 )


ADD_EXECUTABLE( text_mapper ${TEXT_MAPPER_SOURCES} 
    ${TEXT_MAPPER_HEADERS_MOC} 
    ${TEXT_MAPPER_FORMS_HEADERS} )
    
IF( WIN32 )
    SET( TESSERACT_LIBS libtesseract302 )
ELSE( WIN32 )
    SET( TESSERACT_LIBS tesseract )
ENDIF( WIN32 )

TARGET_LINK_LIBRARIES( text_mapper 
    ${QT_LIBRARIES} 
    QVTK 
    ${OpenCV_LIBS} 
    TextMapping 
    ${BOOST_LIBRARIES} 
    ${TESSERACT_LIBS}
    PoissonRecon
    sba )
