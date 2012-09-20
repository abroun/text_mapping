CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

SET( DEFAULT_BUILD_TYPE Release )
SET( DEFAULT_CXX_FLAGS_RELEASE " -Wall -O3 -std=c++0x -fPIC" ) #-fno-implicit-templates" )
SET( BUILD_TYPE_DOC "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." )
SET( CXX_FLAGS_RELEASE_DOC "Flags used by the compiler during release builds (/MD /Ob1 /Oi /Ot /Oy /Gs will produce slightly smaller files" )

IF( DEFINED CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING ${BUILD_TYPE_DOC} )
ELSE()
   SET( CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE} CACHE STRING ${BUILD_TYPE_DOC} )
ENDIF()

IF( DEFINED CMAKE_CXX_FLAGS_RELEASE )
   SET( CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} CACHE STRING ${CXX_FLAGS_RELEASE_DOC} )
ELSE()
   SET( CMAKE_CXX_FLAGS_RELEASE ${DEFAULT_CXX_FLAGS_RELEASE} CACHE STRING ${CXX_FLAGS_RELEASE_DOC} )
ENDIF()


PROJECT( text_mapping )

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# External dependencies
INCLUDE( FindPkgConfig )

FIND_PACKAGE( VTK REQUIRED )
INCLUDE( ${VTK_USE_FILE} )

SET( Boost_USE_STATIC_LIBS OFF )
SET( Boost_USE_MULTITHREADED ON ) 
SET( Boost_USE_STATIC_RUNTIME OFF )
FIND_PACKAGE( Boost 1.46.1 REQUIRED COMPONENTS system filesystem )
FIND_PACKAGE( Eigen3 REQUIRED )

IF(	WIN32 )
	# We remove the security warnings as we're not writing code for production, just for researchers
	ADD_DEFINITIONS( -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS )
ENDIF()

#SET( EIGEN_INCLUDE_DIR "" CACHE PATH "Headers for the eigen library" )

#SET( CMAKE_BUILD_TYPE 
#SET( CMAKE_CXX_FLAGS " -Wall -O3 -march=native -std=c++0x" ) # Overrides default. May not be the best way to set this...

# Main library
INCLUDE_DIRECTORIES(
    .
    ${EIGEN3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include )

LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
	)

ADD_LIBRARY( TextMapping
    src/letter.cpp
    src/text_map.cpp
    src/utilities.cpp
    src/vtk/vtk_text_map_source.cpp
    )

TARGET_LINK_LIBRARIES( 
	TextMapping
    ${Boost_LIBRARIES}
	vtkRendering
    )

IF(UNIX)
	INSTALL( TARGETS TextMapping
			ARCHIVE DESTINATION lib )
	INSTALL( DIRECTORY include/text_mapping DESTINATION include
			FILES_MATCHING PATTERN "*.h" )
ENDIF()

# Python interface using SWIG
FIND_PACKAGE( SWIG )
IF( SWIG_FOUND )
	INCLUDE( ${SWIG_USE_FILE} )

	FIND_PACKAGE( PythonInterp )
	FIND_PACKAGE( PythonLibs )

	EXECUTE_PROCESS( COMMAND ${PYTHON_EXECUTABLE} --version
				  ERROR_VARIABLE PYTHON_VERSION_FULL
				  OUTPUT_STRIP_TRAILING_WHITESPACE )

	STRING(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
	IF(UNIX)
		SET(PYTHON_PLUGIN_INSTALL_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages/TextMapping )
		SET(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages)
	ENDIF()
	IF(WIN32)
		GET_FILENAME_COMPONENT(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
		SET(PYTHON_PLUGIN_INSTALL_PATH "${PYTHON_PATH}/Lib/site-packages/TextMapping")
		SET(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages")
	ENDIF()

	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
	
	SET(CMAKE_SWIG_FLAGS "")

	ADD_SUBDIRECTORY(interfaces/python)
	SET_SOURCE_FILES_PROPERTIES(interfaces/python/TextMapping.i PROPERTIES CPLUSPLUS ON)
	SWIG_ADD_MODULE( TextMapping python interfaces/python/TextMapping.i )
	SWIG_LINK_LIBRARIES( TextMapping ${PYTHON_LIBRARIES} TextMapping )
	set_source_files_properties(${swig_generated_file_fullname} PROPERTIES 
					  COMPILE_FLAGS "-fno-strict-aliasing -w" ) 
					  
	INSTALL( FILES ${CMAKE_BINARY_DIR}/${outdir}/_TextMapping.so
          DESTINATION ${PYTHON_PACKAGES_PATH} )
	INSTALL( FILES ${CMAKE_BINARY_DIR}/${outdir}/TextMapping.py
			  DESTINATION ${PYTHON_PACKAGES_PATH} )
ENDIF()

# Add a target to generate API documentation with Doxygen
FIND_PACKAGE( Doxygen )
IF( DOXYGEN_FOUND )

    CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY )
    ADD_CUSTOM_TARGET( doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

ENDIF( DOXYGEN_FOUND )

#SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
#SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add in applications
ADD_SUBDIRECTORY( apps/camera_apps )
ADD_SUBDIRECTORY( apps/manual_map_builder )
ADD_SUBDIRECTORY( apps/text_mapper )
ADD_SUBDIRECTORY( apps/windows_kinect_grabber )
