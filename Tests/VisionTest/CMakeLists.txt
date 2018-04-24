#miscellaneous
cmake_minimum_required(VERSION 2.8)
project( VisionTest )
set( CMAKE_CXX_STANDARD 14)

#OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#include relevant BachelorPro directories
include_directories( $ENV{BACH_ROOT} )
include_directories( $ENV{TIAFC_ROOT} )
include_directories( $ENV{COORDS_ROOT} )
include_directories( $ENV{ANALYT_ROOT} )
include_directories( $ENV{VISION_ROOT} )

#add relevant BachelorPro libraries
add_library( TIAFC $ENV{TIAFC_ROOT}/TIAFC.cpp)
add_library( Vision $ENV{VISION_ROOT}/Vision.cpp)
add_library( Coords $ENV{COORDS_ROOT}/Coords.cpp)
add_library( AnalytGrasp $ENV{ANALYT_ROOT}/AnalytGrasp.cpp)

#make source.cpp
add_executable( source source.cpp )

#link AnalytGrasp's dependecies to it
target_link_libraries( AnalytGrasp ${OpenCV_LIBS} )
target_link_libraries( AnalytGrasp Coords)
target_link_libraries( AnalytGrasp TIAFC)

#link TIAFC's dependecies to it
target_link_libraries( TIAFC ${OpenCV_LIBS} )
target_link_libraries( TIAFC Coords)

#link Vision's dependecies to it
target_link_libraries( Vision TIAFC )
target_link_libraries( Vision AnalytGrasp)

#link source's dependecies to it
target_link_libraries( source Vision )

