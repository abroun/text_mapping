

SET( POISSON_RECON_SOURCES 
    src/CmdLineParser.cpp
    src/Factor.cpp
    src/Geometry.cpp
	src/MarchingCubes.cpp
	src/ply.cpp
	src/plyfile.cpp
	src/Time.cpp
	src/MultiGridOctest.cpp )
	
ADD_EXECUTABLE( poisson_recon ${POISSON_RECON_SOURCES} )

TARGET_LINK_LIBRARIES( poisson_recon gomp )

