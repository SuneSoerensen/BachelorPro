cmake_minimum_required(VERSION 2.8)
project( the_project )
set( CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_library( TIAFC TIAFC.cpp)
add_library( Vision Vision.cpp)
add_library( Coords Coords.cpp)
add_library( AnalytGrasp AnalytGrasp.cpp)

add_executable( source source.cpp )

target_link_libraries( AnalytGrasp ${OpenCV_LIBS} )
target_link_libraries( AnalytGrasp Coords)

target_link_libraries( TIAFC ${OpenCV_LIBS} )
target_link_libraries( TIAFC Coords)

target_link_libraries( Vision TIAFC )
target_link_libraries( Vision AnalytGrasp)

target_link_libraries( source Vision )
