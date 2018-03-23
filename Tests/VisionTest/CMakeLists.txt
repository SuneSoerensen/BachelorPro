cmake_minimum_required(VERSION 2.8)
project( VisionTest )
set( CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( $ENV{BACH_ROOT} )
include_directories( $ENV{TIAFC_ROOT} )
include_directories( $ENV{COORDS_ROOT} )
include_directories( $ENV{ANALYT_ROOT} )
include_directories( $ENV{VISION_ROOT} )

add_library( TIAFC $ENV{TIAFC_ROOT}/TIAFC.cpp)
add_library( Vision $ENV{VISION_ROOT}/Vision.cpp)
add_library( Coords $ENV{COORDS_ROOT}/Coords.cpp)
add_library( AnalytGrasp $ENV{ANALYT_ROOT}/AnalytGrasp.cpp)

add_executable( source source.cpp )

target_link_libraries( AnalytGrasp ${OpenCV_LIBS} )
target_link_libraries( AnalytGrasp Coords)

target_link_libraries( TIAFC ${OpenCV_LIBS} )
target_link_libraries( TIAFC Coords)

target_link_libraries( Vision TIAFC )
target_link_libraries( Vision AnalytGrasp)

target_link_libraries( source Vision )
