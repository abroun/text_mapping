
INCLUDE_DIRECTORIES(
    .
    ${CMAKE_CURRENT_SOURCE_DIR}/include/poisson_surface_reconstruction )


SET( POISSON_RECON_SOURCES 
    src/CmdLineParser.cpp
    src/Factor.cpp
    src/Geometry.cpp
	src/MarchingCubes.cpp
	src/ply.cpp
	src/plyfile.cpp
	src/Time.cpp )

# Poisson reconstruction library
ADD_LIBRARY( PoissonRecon ${POISSON_RECON_SOURCES} )

TARGET_LINK_LIBRARIES( PoissonRecon gomp )

# Stand alone executable	
ADD_EXECUTABLE( poisson_recon src/MultiGridOctest.cpp )

TARGET_LINK_LIBRARIES( poisson_recon PoissonRecon )


