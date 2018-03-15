# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project.
PROJECT(ArmCamTest)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Same as above, but for robworkhardware
SET(RobWorkHardware_DIR ~/RobWork/RobWorkHardware/cmake)
FIND_PACKAGE(RobWorkHardware REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORKHARDWARE_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKHARDWARE_LIBRARY_DIRS} )

#Same, but for openCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#Same as above, but for bachelorPro
include_directories($ENV{BACH_ROOT})
include_directories($ENV{URCON_ROOT})
include_directories($ENV{VISION_ROOT})
include_directories($ENV{TIAFC_ROOT})
include_directories($ENV{COORDS_ROOT})


# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
 SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# Add your classes REMEMBER TO LINK DEPENDENCIES!
add_library(URControl $ENV{URCON_ROOT}/URControl.cpp)
add_library(TIAFC $ENV{TIAFC_ROOT}/TIAFC.cpp)
add_library(Vision $ENV{VISION_ROOT}/Vision.cpp)
add_library(Coords $ENV{COORDS_ROOT}/Coords.cpp)

# And now we add any targets that we want
add_executable( source source.cpp )

#Link all libraries and classes
target_link_libraries(source ${ROBWORK_LIBRARIES})
target_link_libraries(source ${ROBWORKHARDWARE_LIBRARIES})
target_link_libraries(source URControl)
target_link_libraries(source Vision )

target_link_libraries(URControl ${ROBWORK_LIBRARIES})
target_link_libraries(URControl ${ROBWORKHARDWARE_LIBRARIES})

target_link_libraries( TIAFC Coords)
target_link_libraries( Vision TIAFC )
target_link_libraries( TIAFC ${OpenCV_LIBS} )
