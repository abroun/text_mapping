INCLUDE_DIRECTORIES(
    .
    ${CMAKE_CURRENT_SOURCE_DIR}/include )

SET( SBA_RECON_SOURCES 
    src/bpcg.cpp
    src/sba.cpp
    src/spa.cpp
    src/spa2d.cpp
	src/csparse.cpp
	src/proj.cpp
	src/node.cpp
	src/sba_file_io.cpp )

ADD_LIBRARY( sba ${SBA_RECON_SOURCES} )

TARGET_LINK_LIBRARIES( sba blas lapack cholmod cxsparse )

SET_TARGET_PROPERTIES( sba PROPERTIES
    COMPILE_FLAGS "-DSBA_CHOLMOD -Wunused-but-set-variable"
)


