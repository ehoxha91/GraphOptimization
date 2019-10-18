CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( optimizeCCNY ) #project name

SET(CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE Release  )
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Link the directory
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

#Save header files here
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

#Will build this as part of the project too.
#Should contain cmake file too.
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )
